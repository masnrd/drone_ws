from typing import Dict, Any, Union
from enum import IntEnum
import struct
from math import isnan
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.task import Future
from .maplib import LatLon
from .pathfinder import PathfinderState
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleGlobalPosition, VehicleLocalPosition, VehicleCommandAck, TrajectorySetpoint, VehicleControlMode, VehicleStatus, BatteryStatus
from mc_interface_msgs.msg import Ready
from mc_interface_msgs.srv import Command, Status

""" CONSTANTS """
# Intervals
CYCLE_INTERVAL = 0.1
HB_FC_INTERVAL = 0.1
HB_MC_INTERVAL = 0.1
IDLE_MC_PING_INTERVAL = 30  # How long to spend in IDLE state before prodding MC
IDLE_MC_PING_MAX = 5        # How many times to prod MC until we log a warning

# These control how many failed heartbeats is necessary to determine a loss of connection
FC_CONNECTED_TIMEOUT = 1
MC_CONNECTED_TIMEOUT = 3
CONN_FC_TIMEOUT = 60  # After this timeout, drone has failed to connect to flight controller

# Default constants for the drone
DEFAULT_DRONE_ID = 69
DEFAULT_DIST_FROM_GROUND = 5
DEFAULT_RCH_THRESH = 1  # Distance in metres from a point to consider it 'reached'.

""" HELPER METHODS/FUNCTIONS """
class DroneState(IntEnum):
    """ Current state of the drone """
    INIT    = 0
    CONN_FC = 1  # Drone is connecting to FC (Flight Controller)
    PREP_FC = 2  # Drone is preparing FC (i.e. arming and taking off)
    CONN_MC = 3  # Drone is connecting to MC (Mission Control)
    IDLE    = 4  # Drone is connected, waiting for instructions
    TRAVEL  = 5  # Drone is moving to a position
    SEARCH  = 6  # Drone is searching a sector
    EXIT    = 7  # Drone is landing and ending its state.

class CommandId(IntEnum):
    RTB           = 0  # RTB(): Force RTB. No further commands will be accepted.
    SEARCH_SECTOR = 1  # SEARCH_SECTOR(start, prob_map): Searches a specific sector
    MOVE_TO       = 2  # MOVE_TO(pos): Go to a specific lat/lon.

class UnpackedCommand:
    """
    A Command message from mission control, unpacked for the drone to understand.
    - `cmd_id`: The CommandId.
    - `dest_pos`: The destination position as a LatLon.
        - In the case of RTB and MOVE_TO, this will be the base position and the target position respectively.
        - In the case of SEARCH_SECTOR, this is the start location of the sector.
    - `next_state`: The next state.
    - `args`: A dictionary containing arguments to be handled by the drone for that specific command.
    """
    def __init__(self, cmd: Command):
        self.cmd_id:CommandId = CommandId(cmd.cmd_id)
        self.args: Dict[str, Any] = {}

        # Unpack command
        cmd_data = b''.join(cmd.cmd_data)
        coords = struct.unpack("!ff", cmd_data)
        self.dest_pos = LatLon(coords[0], coords[1])
        if self.cmd_id == CommandId.RTB:
            self.next_state = DroneState.EXIT
        elif self.cmd_id == CommandId.SEARCH_SECTOR:
            self.next_state = DroneState.SEARCH
            self.args["prob_map"] = None
        elif self.cmd_id == CommandId.MOVE_TO:
            self.next_state = DroneState.IDLE

""" DRONE ROS2 Node """
class DroneNode(Node):
    """
    State Information:
    - cycle_timer: Timer that performs actions based on state for the current cycle
    - cycles: Number of cycles so far.
    - `ref_latlon`: Reference Lat/Lon for NED Local World Frame
    - `cur_latlon`: Current Lat/Lon of drone
    - `tgt_latlon`: Target Lat/Lon of drone, published as PositionXY and only in the TRAVEL, SEARCH and RTB states.

    Flight Controller Interaction:
    - fc_hb_timer: Timer that sends heartbeat messages to the flight controller. We run this concurrently with the cycle timer.
    - fc_cycles: Number of heartbeats sent to the flight controller so far.
    - Subscriptions:
        - `sub_vehglobpos`
        - `sub_vehlocpos`
        - `sub_vehcmdack`
        - `sub_batstatus`
    - Publishers:
        - `pub_trajsp`
        - `pub_vehcom`
        - `pub_ocm`
    
    Mission Control Interaction:
    - `mc_last_rtt`: Most recent RTT with Mission Control -- needed for MC to compute the estimated RTT.
    - `mc_last_cmd_id`: Most recent command ID from Mission Control
    - Publishers:
        - `pub_mc_ready`: This acts as the 'heartbeat' message with mission control.
    - Service Clients:
        - `cli_mc_status`
    - Service Servers:
        - `srv_mc_cmd`
    """

    def __init__(self):
        super().__init__("drone")

        # Obtain drone ID
        ## For ROS2 Foxy
        self.declare_parameter("droneId", -1)
        drone_id = self.get_parameter("droneId").get_parameter_value().integer_value
        if drone_id == -1:
            drone_id = None

        # Initialise drone node
        self.drone_state = DroneState.INIT
        self.drone_id = drone_id if drone_id is not None else DEFAULT_DRONE_ID
        self.pathfinder = None
        self.cur_command: Union[UnpackedCommand, None] = None
        self.cur_latlon: Union[LatLon, None] = None
        self.tgt_latlon: Union[LatLon, None] = None
        self.ref_latlon: Union[LatLon, None] = None
        self.cur_alt, self.tgt_alt = 0.0, 0.0
        self.tgt_dfg = DEFAULT_DIST_FROM_GROUND  # Target distance from ground
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Initialise publishers, subscribers for this drone
        ## Flight Controller
        self.fc_sys_id = None
        self.fc_com_id = None
        self.sub_vehstatus= self.create_subscription(
            VehicleStatus,
            f"/fmu/out/vehicle_status",
            self.fc_recv_vehstatus,
            self.qos_profile,
        )
        self.sub_vehglobpos = self.create_subscription(
            VehicleGlobalPosition,
            f"/fmu/out/vehicle_global_position",
            self.fc_recv_vehglobpos,
            self.qos_profile,
        )
        self.sub_vehlocpos = self.create_subscription(
            VehicleLocalPosition,
            f"/fmu/out/vehicle_local_position",
            self.fc_recv_vehlocpos,
            self.qos_profile,
        )
        self.sub_batstatus = self.create_subscription(
            BatteryStatus,
            f"/fmu/out/battery_status",
            self.fc_recv_batstatus,
            self.qos_profile,
        )
        self.pub_vehcom = self.create_publisher(
            VehicleCommand,
            f"/fmu/in/vehicle_command",
            self.qos_profile,
        )
        self.pub_ocm = self.create_publisher(
            OffboardControlMode,
            f"/fmu/in/offboard_control_mode",
            self.qos_profile,
        )
        self.pub_trajsp = self.create_publisher(
            TrajectorySetpoint,
            f"/fmu/in/trajectory_setpoint",
            self.qos_profile,
        )
        ## Mission Control
        self.pub_mc_ready = self.create_publisher(
            Ready,
            f"/mc_{drone_id}/mc/out/ready",
            self.qos_profile,
        )
        self.cli_mc_status = self.create_client(
            Status,
            f"/mc_{drone_id}/mc/srv/status",
        )
        self.srv_mc_cmd = self.create_service(
            Command,
            f"/mc_{drone_id}/mc/srv/cmd",
            self.mc_handle_command,
        )

        # Cycle Information
        self.cycles = 0
        self.cycle_timer = self.create_timer(CYCLE_INTERVAL, self.drone_run)
        self.cycle_error_str = ""
        self.idle_cycles = 0
        self.idle_ping_count = 0

        ## FC
        self.fc_cycles = None
        self.fc_hb_timer = None  # Timer object to send heartbeats to FC
        self.battery_percentage = -1.0
        self.battery_secondsleft = 0.0
        self.fc_armed = False
        self.fc_nav_mode = -1  # 14 if offboard

        ## MC
        self.mc_connected = False
        self.mc_cycles = None
        self.mc_hb_timer = None  # Timer object to send heartbeats to MC
        self.mc_dropped_count = 0
        self.mc_last_rtt = 0.0
        self.mc_last_cmd_id = -1
        self._mc_prev_fut: Union[Future, None] = None

    # State Management
    def change_state(self, new_state: DroneState, msg: str = ""):
        if self.drone_state == DroneState.EXIT:
            self.log(f"Ignore state change to {DroneState(new_state).name}, drone is in EXIT state.")
            return
        
        if len(msg) == 0:
            self.log(f"State change from {self.drone_state.name} to {new_state.name}.")
        else:
            self.log(f"State change from {self.drone_state.name} to {new_state.name}: {msg}")
        
        self.drone_state = new_state

    def drone_run(self):
        """ Main loop for the Drone """
        if self.drone_state == DroneState.INIT:
            self.drone_run_init()
        elif self.drone_state == DroneState.CONN_FC:
            self.drone_run_conn_fc()
        elif self.drone_state == DroneState.PREP_FC:
            self.drone_run_prep_fc()
        elif self.drone_state == DroneState.CONN_MC:
            self.drone_run_conn_mc()
        elif self.drone_state == DroneState.IDLE:
            self.drone_run_idle()
        elif self.drone_state == DroneState.TRAVEL:
            self.drone_run_travel()
        elif self.drone_state == DroneState.SEARCH:
            self.drone_run_search()
        elif self.drone_state == DroneState.EXIT:
            self.drone_run_exit()
        else:
            self.error(f"Invalid drone state {self.drone_state}")
        
        self.cycles += 1

    def fc_run(self):
        """ FC Heartbeat """

        # PX4 FC must receive the stream of VehCmd messages for at least a second.
        if self.fc_cycles <= (CONN_FC_TIMEOUT / HB_FC_INTERVAL):
            # Change mode to Offboard Mode
            self.fc_publish_vehiclecommand(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

            ## Arm the flight controller
            self.fc_publish_vehiclecommand(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        else:
            if self.drone_state in [DroneState.CONN_FC, DroneState.PREP_FC]:
                # Here, we've sent at least a second's worth of messages.
                # Yet, the drone has NOT been armed (or at least has not sent a positive ACK through VehicleCommandAck)
                self.error(f"Failed to arm drone after {CONN_FC_TIMEOUT} seconds.")
                return
            
            if not self.fc_armed:
                self.error(f"Drone disconnected from FC.")
                return

        self.fc_cycles += 1
        self.fc_publish_heartbeat()

    def mc_run(self):
        """ MC Heartbeat """

        # Determine if the previous request was dropped
        if self._mc_prev_fut is None or not self._mc_prev_fut.done():
            # Either we haven't sent a request, or we're still waiting (i.e. assume MC dropped message)
            if self._mc_prev_fut is not None:
                self._mc_prev_fut.cancel()

            self.mc_dropped_count += 1
        else:
            # Previous request was received, not dropped
            self.mc_dropped_count = 0
            self.mc_connected = True
        
        if self.mc_dropped_count >= MC_CONNECTED_TIMEOUT:
            # LOST connection to MC
            self.destroy_timer(self.mc_hb_timer)
            self.mc_hb_timer = None
            self.mc_connected = False
            if self.drone_state != DroneState.EXIT:
                self.change_state(DroneState.CONN_MC, f"Lost connection to MC.")
            return
        
        self._mc_prev_fut = self.mc_request_status()
        self._mc_prev_fut.add_done_callback(self.mc_response_status)

    # States
    def drone_run_init(self):
        """ (Re-)Initialise drone """
        self.cur_latlon, self.ref_latlon, self.tgt_latlon = None, None, None
        self.cur_alt, self.tgt_alt = 0, 0
        self.fc_sys_id, self.fc_com_id = None, None

        # Connect to FC by attempting to get a fix on what our local position is
        self._connfc_check_count = 0
        self.change_state(DroneState.CONN_FC)

    def drone_run_conn_fc(self):
        """ Connect to FC """
        if self._connfc_check_count == (CONN_FC_TIMEOUT // CYCLE_INTERVAL):
            self.error(f"Could not obtain reference point or system ID after {CONN_FC_TIMEOUT} seconds.")
            return
        
        # Transition to next state only if we've established ref_pt
        if self.ref_latlon is None or self.cur_latlon is None or self.fc_sys_id is None or self.fc_com_id is None:
            self._connfc_check_count += 1
            return
        
        # Here, we've established ref_pt and can proceed to prepare the FC
        # self.destroy_subscription(self._connfc_sub_vehlocpos)
        self.change_state(DroneState.PREP_FC, f"Ref LatLon/Cur LatLon:{self.ref_latlon}/{self.cur_latlon}. System ID/Component Id: {self.fc_sys_id}/{self.fc_com_id}")

    def drone_run_prep_fc(self):
        """ Initialise FC by ARMING it and taking off """
        self.fc_cycles = 0
        if self.fc_hb_timer is None:
            self.fc_hb_timer = self.create_timer(HB_FC_INTERVAL, self.fc_run)

        # Transition to next state if drone is armed and in the air
        if self.fc_armed:
            if self.cur_alt < self.tgt_alt - 1:
                self.fc_publish_takeoff()
            else:
                self.change_state(DroneState.CONN_MC, f"FC prepared, taking off.")

    def drone_run_conn_mc(self):
        """ Connect to MC """
        if self.mc_hb_timer is None:
            self.mc_cycles = 0
            self.mc_dropped_count = 0
            self.mc_hb_timer = self.create_timer(HB_MC_INTERVAL, self.mc_run)

        if self.mc_connected:
            self.idle_cycles = 0
            self.change_state(DroneState.IDLE)

    def drone_run_idle(self):
        """ Conduct checks in idle mode """
        if self.idle_cycles == 0:
            # Ping if we're entering the IDLE state
            self.mc_publish_ready()
            self.tgt_latlon = None

        self.idle_cycles += 1
        if self.idle_cycles >= (IDLE_MC_PING_INTERVAL / CYCLE_INTERVAL):
            # Ping, if waited long enough
            self.idle_cycles = 0
            self.idle_ping_count += 1
            self.mc_publish_ready()

        if self.idle_ping_count >= IDLE_MC_PING_MAX:
            self.log(f"Warning: No instructions from MC after {self.idle_ping_count} pings.")
            self.idle_ping_count = 0

    def drone_run_travel(self):
        # Check if we've reached
        reached = (self.tgt_latlon is None) or (self.tgt_latlon.distFromPoint(self.cur_latlon) <= DEFAULT_RCH_THRESH)
        if reached:
            # Change state to the next state. Any necessary arguments should have been processed when the command was first received.
            self.change_state(self.cur_command.next_state, f"Reached {self.tgt_latlon}.")
        else:
            self.fc_publish_trajectorysetpoint()

    def drone_run_search(self):
        # Check if we've reached
        reached = (self.tgt_latlon is None) or (self.tgt_latlon.distFromPoint(self.cur_latlon) <= DEFAULT_RCH_THRESH)
        if reached:
            # Get next waypoint
            if self.pathfinder is None:
                self.error("In search mode, but pathfinder is not defined.")
                return
            
            self.tgt_latlon = self.pathfinder.get_next_waypoint(self.tgt_latlon)
            if self.tgt_latlon is None:
                self.idle_cycles = 0
                self.change_state(DroneState.IDLE, f"Completed search.")
        else:
            self.fc_publish_trajectorysetpoint()

    def drone_run_exit(self):
        self.fc_publish_vehiclecommand(VehicleCommand.VEHICLE_CMD_NAV_LAND)

        # Only exit once the FC disconnects due to landing
        if not self.fc_armed:
            self.log("Landed. Exiting offboard mode.")
            exit(0)

    # ROS2 functions
    ## Subscriptions
    def fc_recv_vehstatus(self, msg: VehicleStatus):
        self.fc_sys_id = msg.system_id
        self.fc_com_id = msg.component_id
        self.fc_armed = True if msg.arming_state == 2 else False
        self.fc_nav_mode = msg.nav_state

    def fc_recv_vehglobpos(self, msg: VehicleGlobalPosition):
        """ Handle an incoming GlobalPosition message from FC. """
        if isnan(msg.lat) or isnan(msg.lon):
            return
        self.cur_latlon = LatLon(msg.lat, msg.lon)
                
    def fc_recv_batstatus(self, msg: BatteryStatus):
        self.battery_percentage = msg.remaining
        self.battery_secondsleft = msg.time_remaining_s

    def fc_recv_vehlocpos(self, msg: VehicleLocalPosition):
        """
        Handle an incoming LocalPosition message.
        """
        if isnan(msg.ref_lat) or isnan(msg.ref_lon):
            return
        self.ref_latlon = LatLon(msg.ref_lat, msg.ref_lon)
        
        # Set tgt_alt, based on distance from ground
        self.cur_alt = -msg.z
        self.tgt_alt = self.cur_alt - msg.dist_bottom + self.tgt_dfg

    # --- PUBLISHERS ---
    def fc_publish_heartbeat(self):
        msg = OffboardControlMode()
        msg.position, msg.velocity, msg.acceleration, msg.body_rate = True, False, False, False
        msg.timestamp = self.clock_microseconds()
        self.pub_ocm.publish(msg)

    def fc_publish_vehiclecommand(self, cmd: int, p1: float = 0.0, p2: float = 0.0, p3: float = 0.0):
        msg = VehicleCommand()
        msg.command = cmd
        msg.target_system, msg.target_component = self.fc_sys_id, self.fc_com_id
        msg.source_system, msg.source_component = 0, 0
        msg.from_external = True
        msg.param1, msg.param2, msg.param3 = p1, p2, p3
        msg.timestamp = self.clock_microseconds()
        self.pub_vehcom.publish(msg)

    def fc_publish_takeoff(self):
        tgt_xy = self.cur_latlon.toXY(self.ref_latlon)  # TrajSP takes only XY relative to the FC's reference point.
        msg = TrajectorySetpoint()
        msg.position = [tgt_xy.x, tgt_xy.y, -self.tgt_alt]
        msg.timestamp = self.clock_microseconds()
        self.pub_trajsp.publish(msg)

    def fc_publish_trajectorysetpoint(self):
        tgt_xy = self.tgt_latlon.toXY(self.ref_latlon)  # TrajSP takes only XY relative to the FC's reference point.
        msg = TrajectorySetpoint()
        msg.position = [tgt_xy.x, tgt_xy.y, -self.tgt_alt]
        msg.timestamp = self.clock_microseconds()
        self.pub_trajsp.publish(msg)

    def mc_publish_ready(self):
        msg = Ready()
        msg.drone_id = self.drone_id
        self.pub_mc_ready.publish(msg)

    # --- CLIENTS ---
    def mc_request_status(self) -> Future:
        # Note: This is sending the drone's current status as a request, not sending a status request
        msg = Status.Request()
        msg.drone_id, msg.drone_mode = self.drone_id, self.drone_state
        msg.timestamp = self.clock_microseconds()
        msg.last_rtt = float(self.mc_last_rtt)
        msg.battery_percentage = float(self.battery_percentage)
        msg.battery_secondsleft = float(self.battery_secondsleft)
        if self.cur_latlon is None:
            msg.lat, msg.lon = float('nan'), float('nan')
        else:
            msg.lat, msg.lon = self.cur_latlon.lat, self.cur_latlon.lon
        return self.cli_mc_status.call_async(msg)

    def mc_response_status(self, fut: Future):
        if fut.cancelled():
            return
        status_ack = fut.result()

        # Obtain RTT in seconds (from microseconds)
        cur_ts  = self.clock_microseconds() / pow(10, 6)
        prev_ts = status_ack.status_timestamp / pow(10, 6)
        self.mc_last_rtt = cur_ts - prev_ts

        #TODO: should we check last command ID?
    
    # --- SERVICES ---
    def mc_handle_command(self, request: Command.Request, response: Command.Response):
        self.mc_last_cmd_id = request.cmd_id

        self.cur_command = UnpackedCommand(request)
        self.log(f"Received {self.cur_command.cmd_id.name}.")
        self.tgt_latlon = self.cur_command.dest_pos
        
        if self.cur_command.cmd_id == CommandId.SEARCH_SECTOR:
            # Initialise pathfinder
            self.pathfinder = PathfinderState(self.cur_command.dest_pos, self.cur_command.args["prob_map"])

        self.change_state(DroneState.TRAVEL, f"Received new command: {self.cur_command.cmd_id.name}")
        response.drone_id, response.cmd_id = self.drone_id, request.cmd_id
        return response

    # Helper Methods
    def clock_microseconds(self) -> int:
        """ Returns the current clock in microseconds. """
        return self.get_clock().now().nanoseconds // 1000
    
    def log(self, msg: str):
        msg = f"DRONE {self.drone_id}: {msg}"
        self.get_logger().info(msg)

    def error(self, msg: str):
        self.change_state(DroneState.EXIT, f"Exiting due to error {msg}")

def main(args=None):
    rclpy.init(args=args)
    drone_node = DroneNode()

    rclpy.spin(drone_node)

    drone_node.destroy_node()

if __name__ == '__main__':
    main()