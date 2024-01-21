import struct
from math import isnan
from enum import IntEnum
from typing import Dict, List, Any
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.task import Future
from rclpy.exceptions import ParameterUninitializedException
from .maplib import LatLon
from .pathfinder import PathfinderState
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleGlobalPosition, VehicleLocalPosition, VehicleCommandAck, TrajectorySetpoint, VehicleControlMode, VehicleStatus
from mc_interface_msgs.msg import Ready
from mc_interface_msgs.srv import Command, Status

DEFAULT_DRONE_ID = 69
CYCLE_INTERVAL = 0.1   # 100ms between each cycle.
FC_HEARTBEAT_INTERVAL = 0.1
MC_HEARTBEAT_INTERVAL = 1.0
DEFAULT_TGT_ALTITUDE = 5
DEFAULT_THRESHOLD = 1  # Distance in metres from a point to know that we've 'reached' it.

IDLE_PING_INTERVAL = 30 # Amount of time spent in IDLE mode before drone prods mission control
MAX_IDLE_PINGS = 5      # Maximum times the drone prods mission control.

CONNFC_TIMEOUT = 15  # After this timeout, drone has failed to connect to flight controller.
FC_ARM_TIMEOUT = 1   # After this timeout, we conclude that the drone has failed to arm.

MC_TIMEOUT = 3  # Number of times we miss a response from mission control to decide we've lost connection.

class DroneMode(IntEnum):
    ERROR      = -99 # Drone experienced an irrecoverable error
    RTB        = -2  # Drone is returning to MC
    IDLE       = -1  # Drone is idle
    INIT       = 0   # Complete initialisation
    CONNECT_FC = 1   # Connecting to Flight Controller (FC)
    INIT_FC    = 2   # Initialisation of subscriptions and publishers
    CONNECT_MC = 3   # Connecting to Mission Control (MC)
    TRAVEL     = 4   # Moving to Sector
    SEARCH     = 5   # Searching Sector

class CommandId(IntEnum):
    RTB           = 0  # Force RTB. No further commands will be accepted.
    SEARCH_SECTOR = 1
    MOVE_TO       = 2  # Go to a specific lat/lon.

def unpack_command(cmd: Command) -> Dict[str, Any]:
    """ Unpacks a Command from the Mission Control. """
    cmd_id = cmd.cmd_id
    cmd_data = b''.join(cmd.cmd_data)

    ret = {"command_id": cmd_id}
    if cmd_id == CommandId.RTB:
        coords = struct.unpack("!ff", cmd_data)
        ret["base_pos"] = LatLon(coords[0], coords[1])
    elif cmd_id == CommandId.SEARCH_SECTOR:
        coords = struct.unpack("!ff", cmd_data)
        ret["sector_start_pos"] = LatLon(coords[0], coords[1])
        ret["sector_prob_map"] = None  #TODO: decide how to encode prob_map for MC
    elif cmd_id == CommandId.MOVE_TO:
        coords = struct.unpack("!ff", cmd_data)
        ret["goto_pos"] = LatLon(coords[0], coords[1])
    
    return ret

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
        - `sub_vehcmdack`
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
        try:
            self.declare_parameter("droneId", rclpy.Parameter.Type.INTEGER)
            drone_id = self.get_parameter("droneId").get_parameter_value().integer_value
        except ParameterUninitializedException:
            drone_id = None

        # Initialise drone node
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.drone_state = DroneMode.INIT
        self.next_state = None   # Used to differentiate between a MOVE_TO command and SEARCH_SECTOR command.
        self.drone_id = drone_id if drone_id is not None else DEFAULT_DRONE_ID
        self.log("Initialised.")

        self.idle_cycles = 0

        # Initialise subs/pubs for this particular drone
        self._drone_ns = f"/px4_{drone_id}" if drone_id is not None else ""
        ## Flight Controller
        self.fc_sys_id = None
        self.fc_com_id = None
        self.sub_vehcmdack = self.create_subscription(
            VehicleCommandAck,
            f"{self._drone_ns}/fmu/out/vehicle_command_ack",
            self.fc_recv_vehcmdack,
            self.qos_profile,
        )
        self.sub_vehglobpos = self.create_subscription(
            VehicleGlobalPosition,
            f"{self._drone_ns}/fmu/out/vehicle_global_position",
            self.fc_recv_vehglobpos,
            self.qos_profile,
        )
        self.sub_vehconmode = self.create_subscription(
            VehicleControlMode,
            f"{self._drone_ns}/fmu/out/vehicle_control_mode",
            self.fc_recv_vehconmode,
            self.qos_profile,
        )
        self.pub_vehcom = self.create_publisher(
            VehicleCommand,
            f"{self._drone_ns}/fmu/in/vehicle_command",
            self.qos_profile,
        )
        self.pub_ocm = self.create_publisher(
            OffboardControlMode,
            f"{self._drone_ns}/fmu/in/offboard_control_mode",
            self.qos_profile,
        )
        self.pub_trajsp = self.create_publisher(
            TrajectorySetpoint,
            f"{self._drone_ns}/fmu/in/trajectory_setpoint",
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

        self.cycles = 0
        self.cycle_timer = self.create_timer(CYCLE_INTERVAL, self.drone_run)
        self.cycle_error_str = ""
        self.idle_cycles = 0
        self.idle_ping_count = 0

        self.fc_cycles = None
        self.fc_hb_timer = None

        self.mc_cycles = None
        self.mc_hb_timer = None
        self.mc_dropped_count = 0
        self.mc_prev_future:Future = None
        self.mc_last_rtt = 0.0
        self.mc_last_cmd_id = -1

    def clock_microseconds(self) -> int:
        """ Returns the current clock in microseconds. """
        return self.get_clock().now().nanoseconds // 1000
    
    def log(self, msg: str):
        print(f"D{self.drone_id}: {msg}")

    def change_state(self, new_state: DroneMode, msg: str = ""):
        #TODO: place state transitions here?
        if self.drone_state == DroneMode.ERROR or self.drone_state == DroneMode.RTB:
            print(f"DRONE: Ignoring state change from {DroneMode(self.drone_state).name} to {DroneMode(new_state).name}, drone is currently ignoring commands.")
            return

        print(f"DRONE: State Change from {DroneMode(self.drone_state).name} to {DroneMode(new_state).name}")
        if len(msg) != 0:
            print(f"DRONE: {msg}")
        self.drone_state = new_state

        if new_state == DroneMode.IDLE:
            self.idle_cycles = 0
            self.idle_ping_count = 0
            self.mc_publish_ready()

    def raise_error(self, error_str: str):
        self.cycle_error_str = error_str
        self.drone_state = DroneMode.ERROR

    def drone_run(self):
        """
        Main loop for the Drone
        Note: Please don't change this to a match case statement -- I'm not sure if 
        the Jetson has Python 3.10 and above.
        """
        if self.drone_state == DroneMode.INIT:
            self.drone_run_init()
        elif self.drone_state == DroneMode.CONNECT_FC:
            self.drone_run_connect_fc()
        elif self.drone_state == DroneMode.INIT_FC:
            self.drone_run_init_fc()
        elif self.drone_state == DroneMode.CONNECT_MC:
            self.drone_run_connect_mc()
        elif self.drone_state == DroneMode.IDLE:
            self.drone_run_idle()
        elif self.drone_state == DroneMode.TRAVEL:
            self.drone_run_travel()
        elif self.drone_state == DroneMode.SEARCH:
            self.drone_run_search()
        elif self.drone_state == DroneMode.RTB:
            self.drone_run_rtb()
        elif self.drone_state == DroneMode.ERROR:
            print(f"DRONE ERROR: {self.cycle_error_str} (Cycle {self.cycles})")
            #TODO: Safe handling of drone
            exit(1)
        else:
            print(f"DRONE ERROR: Unknown drone state {self.drone_state}.")
            #TODO: Safe handling of drone
            exit(1)

        self.cycles += 1

    def drone_run_init(self):
        """ (Re-)Initialise drone """
        self.cur_latlon, self.ref_latlon, self.tgt_latlon = None, None, None
        self.tgt_altitude = DEFAULT_TGT_ALTITUDE

        # Connection to FC
        ## Connect to FC by attempting to get a fix on what our local position is and to get system and component ID
        self._connfc_check_count = 0
        self._connfc_sub_vehlocpos = self.create_subscription(
            VehicleLocalPosition,
            f"{self._drone_ns}/fmu/out/vehicle_local_position",
            self._fc_recv_locpos,
            self.qos_profile,
        )
        self._connfc_sub_vehstatus = self.create_subscription(
            VehicleStatus,
            f"{self._drone_ns}/fmu/out/vehicle_status",
            self._fc_recv_vehstatus,
            self.qos_profile,
        )
        self.change_state(DroneMode.CONNECT_FC)

    def drone_run_connect_fc(self):
        """ Connect to the Flight Controller """
        if self._connfc_check_count == (CONNFC_TIMEOUT // CYCLE_INTERVAL):
            self.raise_error(f"Could not obtain reference point or system ID after {CONNFC_TIMEOUT} seconds.")
            return
        
        # Check if we've received a reference point from our LocalPosition and GlobalPosition subscriptions
        if self.ref_latlon is None or self.cur_latlon is None:
            self._connfc_check_count += 1
            return
        
        # Ensure the system ID and component ID is known
        if self.fc_sys_id is None or self.fc_com_id is None:
            self._connfc_check_count += 1
            return

        # Here, we've established the reference point and can proceed to the INIT_FC stage.
        self.destroy_subscription(self._connfc_sub_vehlocpos)
        self.destroy_subscription(self._connfc_sub_vehstatus)
        self.fc_publish_vehiclecommand(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF)
        self.change_state(DroneMode.INIT_FC, f"Ref LatLon/Cur LatLon: {self.ref_latlon}/{self.cur_latlon}. System ID/Component Id: {self.fc_sys_id}/{self.fc_com_id}")

    def drone_run_init_fc(self):
        """ Initialise Flight Controller. """
        self.fc_cycles = 0
        if self.fc_hb_timer is None:
            self.fc_hb_timer = self.create_timer(FC_HEARTBEAT_INTERVAL, self.fc_run)

        # Note: This state is only complete once the drone is ARMED, which is done in the callback for VehicleCommandAck.

    def drone_run_connect_mc(self):
        """ Connect to Mission Control. """
        if self.mc_hb_timer is None:
            self.mc_cycles = 0
            self.mc_dropped_count = 0
            self.mc_hb_timer = self.create_timer(MC_HEARTBEAT_INTERVAL, self.mc_run)

    def drone_run_idle(self):
        """ Conduct checks in idle mode. """
        self.idle_cycles += 1
        if self.idle_cycles >= (IDLE_PING_INTERVAL / CYCLE_INTERVAL):
            # Ping, if we've waited long enough.
            self.idle_cycles = 0
            self.idle_ping_count += 1
            self.mc_publish_ready()
        
        if self.idle_ping_count >= MAX_IDLE_PINGS:
            # Switch to failure mode and RTB
            #TODO: should not just error out?
            self.raise_error(f"Cannot connect to Mission Control after {MAX_IDLE_PINGS} pings.")
            return

    def drone_run_travel(self):
        """ Travel to a location (without searching) """
        self.fc_publish_trajectorysetpoint()

    def drone_run_search(self):
        """ Search a sector. """
        self.fc_publish_trajectorysetpoint()

    def drone_run_rtb(self):
        """ Return to base. """
        self.fc_publish_trajectorysetpoint()

    def fc_run(self):
        """ Callback to send a heartbeat to the flight controller. """

        # PX4 FC must receive the stream of VehCmd messages for at least a second.
        if self.fc_cycles <= (FC_ARM_TIMEOUT / FC_HEARTBEAT_INTERVAL):
            # Change mode to Offboard Mode
            self.fc_publish_vehiclecommand(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

            ## Arm the flight controller
            self.fc_publish_vehiclecommand(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        else:
            if self.drone_state == DroneMode.INIT_FC:
                # Here, we've sent at least a second's worth of messages.
                # Yet, the drone has NOT been armed (or at least has not sent a positive ACK through VehicleCommandAck)
                self.raise_error(f"Failed to arm drone after {FC_ARM_TIMEOUT} seconds.")
                return
        
        self.fc_cycles += 1
        self.fc_publish_heartbeat()

    def mc_run(self):
        """ Callback to send a heartbeat to mission control. """
        hb_dropped = False
        if self.mc_prev_future is None or not self.mc_prev_future.done():
            # Still waiting for a response from mission control
            hb_dropped = True
            if self.mc_prev_future is not None:
                self.mc_prev_future.cancel()
        else:
            # Previous response was received.
            self.mc_dropped_count = 0

        # Perform actions based on current state
        if self.drone_state == DroneMode.CONNECT_MC:
            # Drone is trying to connect
            if not hb_dropped:
                # Previous heartbeat cycle reached mission control.
                self.change_state(DroneMode.IDLE, "Connected to MC.")
            else:
                self.mc_dropped_count += 1
        elif self.drone_state in [DroneMode.IDLE, DroneMode.TRAVEL, DroneMode.SEARCH]:
            # Connection has already been established
            if hb_dropped:
                # Connection possibly broken, try again
                self.mc_dropped_count += 1
        
        if self.mc_dropped_count >= MC_TIMEOUT:
            self.destroy_timer(self.mc_hb_timer)
            self.mc_hb_timer = None
            if self.drone_state == DroneMode.CONNECT_MC:
                self.raise_error(f"Could not connect to mission control after {MC_TIMEOUT * MC_HEARTBEAT_INTERVAL} seconds.")
                return
            self.change_state(DroneMode.CONNECT_MC, "Lost connection to mission control.")
            #TODO: should store the OLD mode such that we can recover that once we regain connection.
            return
        
        # Send status
        self.mc_prev_future = self.mc_request_status()
        self.mc_prev_future.add_done_callback(self.mc_response_status)


    # --- SUBSCRIPTIONS ---
    def fc_recv_vehglobpos(self, msg: VehicleGlobalPosition):
        """ Handle an incoming GlobalPosition message from FC. """
        if isnan(msg.lat) or isnan(msg.lon):
            return
        self.cur_latlon = LatLon(msg.lat, msg.lon)

        # Update the target latlon if we've reached it
        if self.drone_state in [DroneMode.TRAVEL, DroneMode.SEARCH, DroneMode.RTB]:
            if self.tgt_latlon is None:
                return
            if self.tgt_latlon.distFromPoint(self.cur_latlon) > DEFAULT_THRESHOLD:
                return
            
            if self.drone_state == DroneMode.TRAVEL:
                # Change state depending on next_state
                if self.next_state is None:
                    self.raise_error(f"Reached {self.tgt_latlon} but no next state set.")
                    return
                elif self.next_state == DroneMode.IDLE:
                    self.change_state(DroneMode.IDLE, f"Reached {self.tgt_latlon}.")
                elif self.next_state == DroneMode.SEARCH:
                    self.change_state(DroneMode.SEARCH, f"Reached {self.tgt_latlon}, starting search.")
                    self.tgt_latlon = self.pathfinder.get_next_waypoint(self.tgt_latlon)
            elif self.drone_state == DroneMode.SEARCH:
                self.log(f"Reached {self.tgt_latlon} in SEARCH state.")
                self.tgt_latlon = self.pathfinder.get_next_waypoint(self.tgt_latlon)
                #TODO: add checks for when we've completed the search.
            elif self.drone_state == DroneMode.RTB:
                self.log(f"Reached {self.tgt_latlon} in RTB state.")
                #TODO: add safe exit sequence
                exit(0)

    def fc_recv_vehcmdack(self, msg: VehicleCommandAck):
        """ Handle an incoming VehicleCommandAck message from FC. """
        if msg.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            # This message is acknowledging a request to arm/disarm
            if msg.result != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                return
            
            # Here, the FC has ACCEPTED our request
            if self.drone_state == DroneMode.INIT_FC:
                # We have been asking the FC to arm.
                #TODO: add check that the ack is for an ARMING request
                self.change_state(DroneMode.CONNECT_MC, "Drone has been armed in offboard mode.")
            #TODO: add disarm check
                
    def fc_recv_vehconmode(self, msg: VehicleControlMode):
        if self.drone_state in [DroneMode.RTB, DroneMode.IDLE, DroneMode.CONNECT_MC, DroneMode.TRAVEL, DroneMode.SEARCH]:
            #TODO: safer way is just to take off the drone upon arming.
            if not msg.flag_armed or not msg.flag_control_offboard_enabled:
                # Drone was disarmed
                self.change_state(DroneMode.INIT_FC, "Drone disconnected from FC, reconnecting.")

    def _fc_recv_locpos(self, msg: VehicleLocalPosition):
        """
        Handle an incoming LocalPosition message.
        This is temporary -- the subscription is deleted once we've connected.
        """
        if isnan(msg.ref_lat) or isnan(msg.ref_lon):
            return
        self.ref_latlon = LatLon(msg.ref_lat, msg.ref_lon)
                
    def _fc_recv_vehstatus(self, msg: VehicleStatus):
        """ Handle an incoming VehicleStatus message from FC. """
        self.fc_sys_id = msg.system_id
        self.fc_com_id = msg.component_id

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

    def fc_publish_trajectorysetpoint(self):
        tgt_xy = self.tgt_latlon.toXY(self.ref_latlon)  # TrajSP takes only XY relative to the FC's reference point.
        msg = TrajectorySetpoint()
        msg.position = [tgt_xy.x, tgt_xy.y, -self.tgt_altitude]
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
        msg.last_rtt = self.mc_last_rtt
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

        instruction = unpack_command(request)
        self.log(f"Received {CommandId(instruction['command_id']).name}. Switching to TRAVEL state.")
        if instruction["command_id"] != CommandId.SEARCH_SECTOR:
            self.raise_error(f"Received unimplemented command {CommandId(instruction['command_id']).name}.")
            return
        elif instruction["command_id"] == CommandId.SEARCH_SECTOR:
            start_latlon = instruction["sector_start_pos"]
            prob_map = instruction["sector_prob_map"]

            # Initialise pathfinder
            self.pathfinder = PathfinderState(start_latlon, prob_map)
            self.tgt_latlon = start_latlon
            self.next_state = DroneMode.SEARCH
            self.change_state(DroneMode.TRAVEL)
        
        response.drone_id = self.drone_id
        response.cmd_id = request.cmd_id
        return response

def main(args=None):
    rclpy.init(args=args)
    drone_node = DroneNode()

    rclpy.spin(drone_node)

    drone_node.destroy_node()

if __name__ == '__main__':
    main()