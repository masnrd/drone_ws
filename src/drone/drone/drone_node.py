import rclpy
import struct
from math import isnan
from enum import IntEnum
from threading import Event
from typing import Dict, List
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.task import Future
from .maplib import LatLon, PositionXY
from .pathfinder import PathfinderState
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleGlobalPosition, VehicleLocalPosition, VehicleCommandAck, TrajectorySetpoint
from mc_interface_msgs.msg import Ready
from mc_interface_msgs.srv import Command, Status

CONNFC_CHECK_INTERVAL = 0.5  # Interval to check for the refpoint
CONNFC_TIMEOUT = 15  # Number of seconds until we timeout the refpoint check
IDLE_PING_INTERVAL = 30 #SWITCH TO 30 ONCE SET  # Number of seconds until we ping MC for an update while in IDLE mode
MAX_IDLE_PINGS = 10  # Ping a max of 10 times before we consider it a failure to connect
TIMER_INTERVAL = 0.1  # 100 ms
MC_HEARTBEAT_INTERVAL = 1 # 1 s
MC_MAX_HEARTBEAT_DROPS = 10
DEFAULT_TGT_ALTITUDE = 5

DEFAULT_DRONE_ID = 69  #TODO: fix for multiple drones

class DroneCommandId(IntEnum):
    RTB           = 0  # Force RTB. No further commands will be accepted.
    SEARCH_SECTOR = 1
    MOVE_TO       = 2  # Go to a specific lat/lon.

class DroneCommand:
    """ Interface for a command """
    def __init__(self, command_id: int, command_data: List[bytes]):
        self.command_id = command_id
        self.command_data = b''.join(command_data)

    def unpack_command(self) -> Dict:
        ret = {
            "command_id": self.command_id
        }
        if self.command_id == DroneCommandId.RTB:
            unpacked = struct.unpack("!ff", self.command_data)
            ret["base_pos"] = LatLon(unpacked[0], unpacked[1])
        elif self.command_id == DroneCommandId.SEARCH_SECTOR:
            unpacked = struct.unpack("!ff", self.command_data)
            ret["sector_start_pos"] = LatLon(unpacked[0], unpacked[1])
            ret["sector_prob_map"] = None #TODO: decode np array
        elif self.command_id == DroneCommandId.MOVE_TO:
            unpacked = struct.unpack("!ff", self.command_data)
            ret["goto_pos"] = LatLon(unpacked[0], unpacked[1])

        return ret

class DroneMode(IntEnum):
    RTB        = -2  # Drone is returning to MC
    IDLE       = -1  # Drone is idle
    INIT       = 0   # Complete initialisation
    CONNECT_FC = 1   # Connecting to Flight Controller (FC)
    INIT_FC    = 2   # Initialisation of subscriptions and publishers
    CONNECT_MC = 3   # Connecting to Mission Control (MC)
    TRAVEL     = 4   # Moving to Sector
    SEARCH     = 5   # Searching Sector

class DroneError(Exception):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)

class DroneNode(Node):
    """
    - `ref_latlon`: Reference Lat/Lon for NED Local World Frame
    - `cur_latlon`: Current Lat/Lon of drone
    - `tgt_latlon`: Target Lat/Lon of drone, published as PositionXY and only in the TRAVEL, SEARCH and RTB states.
    - `mc_last_rtt`: Most recent round-trip-time with Mission Control
    - `mc_last_cmd_id`: Most recent command ID from Mission Control
    - Subscriptions:
        - `sub_vehglobpos`
        - `sub_vehcmdack`
    - Publishers:
        - `pub_trajsp`
        - `pub_vehcom`
        - `pub_ocm`
        - `pub_mc_ready` (MC)
    - Service Clients:
        - `cli_mc_status` (MC)
    - Service Servers:
        - `srv_mc_cmd` (MC)
    """

    def __init__(self):
        super().__init__("drone")
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.drone_state = DroneMode.INIT
        self.drone_id = DEFAULT_DRONE_ID

        self.drone_init()

    def drone_init(self):
        """ Drone initialisation """
        print("DRONE: Initialising.")
        if self.drone_state != DroneMode.INIT:
            raise DroneError(f"Expected INIT state, current state is {self.drone_state}")
        
        self.ref_latlon = None
        self.cur_latlon = None
        self.tgt_latlon = None
        self.tgt_altitude = DEFAULT_TGT_ALTITUDE

        # Connection to FC
        self.sub_vehglobpos = None
        self.sub_vehcmdack = None
        self.pub_trajsp = None
        self.pub_vehcom = None
        self.pub_ocm = None
        self.heartbeat_timer = None

        # Connection to MC
        self.pub_mc_ready = None
        self.cli_mc_status = None
        self.srv_mc_cmd = None
        self.mc_heartbeat_timer = None
        self.mc_last_rtt = 0.0
        self.mc_last_cmd_id = -1
        self._mc_last_future = None
        self._mc_heartbeat_dropped_count = 0

        # Miscellaneous
        self.pathfinder = None
        self._counter = None

        # Start subscribers for reference point
        self.ref_latlon = None
        self._connfc_check_count = 0
        self.sub_vehglobpos = self.create_subscription(
            VehicleGlobalPosition,
            "/fmu/out/vehicle_global_position",
            self.fc_recv_vehglobpos,
            self.qos_profile,
        )
        self._connfc_sub_vehlocpos = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._fc_recv_locpos,
            self.qos_profile,
        )
        self.drone_state = DroneMode.CONNECT_FC
        print("DRONE: Connecting to FC...")
        self._connfc_timer = self.create_timer(CONNFC_CHECK_INTERVAL, self.drone_connect_fc)

    def drone_connect_fc(self):
        """ Wait for FC status to reach drone """
        if self.drone_state != DroneMode.CONNECT_FC:
            raise DroneError(f"Expected CONNECT_FC state, current state is {self.drone_state}")
        
        if self._connfc_check_count == (CONNFC_TIMEOUT / CONNFC_CHECK_INTERVAL):
            raise DroneError(f"Could not obtain the reference point after {CONNFC_TIMEOUT} seconds.")

        if self.ref_latlon is None or self.cur_latlon is None:
            self._connfc_check_count += 1
            return

        # Here, we've established the reference point and can proceed to the INIT_FC stage.
        self.destroy_timer(self._connfc_timer)
        self.destroy_subscription(self._connfc_sub_vehlocpos)
        print(f"DRONE: Connected to FC. Reference LatLon: {self.ref_latlon}, Current LatLon: {self.cur_latlon}")
        self.drone_state = DroneMode.INIT_FC
        self.drone_init_fc()

    def _fc_recv_locpos(self, msg: VehicleLocalPosition):
        if isnan(msg.ref_lat) or isnan(msg.ref_lon):
            return
        self.ref_latlon = LatLon(msg.ref_lat, msg.ref_lon)

    def drone_init_fc(self):
        """ Initialise the drone state, arm drone in offboard control mode """
        print("DRONE: Initialising FC.")
        if self.drone_state != DroneMode.INIT_FC:
            raise DroneError(f"Expected INIT_FC state, current state is {self.drone_state}")
        
        self.sub_vehcmdack = self.create_subscription(
            VehicleCommandAck,
            "/fmu/out/vehicle_command_ack",
            self.fc_recv_vehcmdack,
            self.qos_profile,
        )
        self.pub_trajsp = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            self.qos_profile,
        )
        self.pub_vehcom = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            self.qos_profile,
        )
        self.pub_ocm = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            self.qos_profile,
        )

        # Attempt to arm drone in Offboard mode
        self._counter = 0
        self.heartbeat_timer = self.create_timer(TIMER_INTERVAL, self.heartbeat_timer_callback)

    def drone_connect_mc(self):
        """ Establish a connection to Mission Control """
        print("DRONE: Connecting to Mission Control...")
        if self.drone_state != DroneMode.CONNECT_MC:
            raise DroneError(f"Expected CONNECT_MC state, current state is {self.drone_state}")
        
        # Initialise Mission Control communication
        self.pub_mc_ready = self.create_publisher(
            Ready,
            f"/drone_{self.drone_id}/out/ready",
            self.qos_profile,
        )
        self.cli_mc_status = self.create_client(
            Status,
            f"/drone_{self.drone_id}/srv/status",
        )
        self.srv_mc_cmd = self.create_service(
            Command,
            f"/drone_{self.drone_id}/srv/cmd",
            self.mc_recv_command,
        )

        # Attempt to establish connection with MC
        self.mc_heartbeat_timer = self.create_timer(MC_HEARTBEAT_INTERVAL, self.mc_heartbeat_timer_callback)

    def drone_idle(self):
        """ Wait for MC instructions. """
        if self.drone_state != DroneMode.IDLE:
            raise DroneError(f"Expected IDLE state, current state is {self.drone_state}")
        
        # Alert MC of ready state
        self.publish_ready()
        
        # Set timer to wait
        print("DRONE: Waiting for instructions from MC...")
        self._idle_check_count = 0
        self._idle_timer = self.create_timer(IDLE_PING_INTERVAL, self.drone_idle_ping)
    
    def drone_idle_ping(self):
        """ Ping MC for an update """
        if self.drone_state == DroneMode.IDLE:
            self._idle_check_count += 1
            if self._idle_check_count >= MAX_IDLE_PINGS:
                #TODO: switch to failure mode and RTB
                pass

            # Inform MC of ready state
            self.publish_ready()

    def heartbeat_timer_callback(self):
        """ Called to maintain the heartbeat of OffboardControlMessages to the flight controller. """
        if (self._counter <= (1 / TIMER_INTERVAL)):
            # Must receive stream for at least a second
            ## VehicleCommand: Change mode to Offboard Mode
            self.publish_vehcmdmsg(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

            ## VehicleCommand: Arm
            self.publish_vehcmdmsg(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        else:
            # Should be already armed, otherwise fail
            if self.drone_state == DroneMode.INIT_FC:
                raise DroneError(f"Failed to arm drone after {TIMER_INTERVAL}.")
        self._counter += 1
        
        # Publish heartbeat
        hb_msg = OffboardControlMode()
        hb_msg.position = True
        hb_msg.velocity = False
        hb_msg.acceleration = False
        hb_msg.body_rate = False
        hb_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.pub_ocm.publish(hb_msg)

        # Publish tgt position if drone is in TRAVEL, SEARCH or RTB states
        if self.drone_state in [DroneMode.TRAVEL, DroneMode.SEARCH, DroneMode.RTB]:
            tgt_xy = self.tgt_latlon.toXY(self.ref_latlon)
            tsp_msg = TrajectorySetpoint()
            tsp_msg.position = [tgt_xy.x, tgt_xy.y, -self.tgt_altitude]
            tsp_msg.timestamp = self.get_clock().now().nanoseconds // 1000
            self.pub_trajsp.publish(tsp_msg)

    def mc_heartbeat_timer_callback(self):
        """ Called to maintain status updates to Mission Control """
        # Detect if previous heartbeat was dropped
        dropped_heartbeat = False
        if self._mc_last_future is not None and not self._mc_last_future.done():
            dropped_heartbeat = True
            self._mc_last_future.cancel()

        if self.drone_state == DroneMode.CONNECT_MC:
            # Still attempting to connect to MC
            if not dropped_heartbeat:
                # Connection success.
                print("DRONE: Connected to Mission Control.")
                self.drone_state = DroneMode.IDLE
                self.drone_idle()
        elif self.drone_state in [DroneMode.IDLE, DroneMode.TRAVEL, DroneMode.SEARCH]:
            # Drone should be in a state where it has previously already established a connection
            if dropped_heartbeat:
                self._mc_heartbeat_dropped_count += 1
                if self._mc_heartbeat_dropped_count >= MC_MAX_HEARTBEAT_DROPS:
                    #TODO: depends on how independent the drone is -- maybe it's only necessary to reconnect once we're in idle mode
                    print("DRONE: Lost connection to Mission Control.")
                    self.destroy_timer(self.mc_heartbeat_timer)
                    self.drone_state = DroneMode.CONNECT_MC
                    self.drone_connect_mc()
                    return
        elif self.drone_state == DroneMode.RTB:
            # Returning to Base for reset, should not need this any more
            self.destroy_timer(self.mc_heartbeat_timer)
            return
        else:
            print("DRONE ERROR: Received heartbeat response when not connected to MC.")
            return

        status = Status.Request()
        status.drone_id = self.drone_id
        status.drone_mode = self.drone_state
        status.timestamp = self.get_clock().now().nanoseconds // 1000
        status.last_rtt = self.mc_last_rtt
        if self.cur_latlon is None:
            status.lat = float('nan')
            status.lon = float('nan')
        else:
            status.lat = self.cur_latlon.lat
            status.lon = self.cur_latlon.lon
        self._mc_last_future:Future = self.cli_mc_status.call_async(status)
        self._mc_last_future.add_done_callback(self.mc_recv_status_ack)
    
    def mc_recv_status_ack(self, status_future: Future):
        """ Action upon receiving a status ACK """
        if status_future.cancelled():
            return
        status_ack = status_future.result()
        cur_ts = (self.get_clock().now().nanoseconds // 1000) / pow(10, 6)
        prev_ts = status_ack.status_timestamp / pow(10, 6)
        self.mc_last_rtt = cur_ts - prev_ts # RTT in seconds
        self._mc_last_future = None
        self._mc_heartbeat_dropped_count = 0

        if self.mc_last_cmd_id != status_ack.last_cmd_id and self.mc_last_cmd_id != -1:
            # We missed a command somewhere
            #TODO: do something
            print(f"DRONE WARNING: Drone processed last command {self.mc_last_cmd_id}, MC Status_ACK last command {status_ack.last_cmd_id}")
            pass
        
    def mc_recv_command(self, request: Command.Request, response: Command.Response):
        """ Receives a command from MC """
        cmd = DroneCommand(request.cmd_id, request.cmd_data)
        response.drone_id = request.drone_id
        response.cmd_id = request.cmd_id
        self.mc_last_cmd_id = request.cmd_id

        instruction = cmd.unpack_command()
        print(f"DRONE: Received {DroneCommandId(instruction['command_id']).name} instruction from MC. Switching to TRAVEL state.")
        if instruction["command_id"] != DroneCommandId.SEARCH_SECTOR:
            print(f"DRONE WARNING: Received unimplemented command {DroneCommandId(instruction['command_id']).name}")
        elif instruction["command_id"] == DroneCommandId.SEARCH_SECTOR:
            start_latlon = instruction["sector_start_pos"]
            prob_map = instruction["sector_prob_map"]
            self.pathfinder = PathfinderState(start_latlon, prob_map)
            self.tgt_latlon = start_latlon
            self.destroy_timer(self._idle_timer)
            self.drone_state = DroneMode.TRAVEL

        return response

    def fc_recv_vehglobpos(self, msg: VehicleGlobalPosition):
        # Received VehicleGlobalPosition from FC
        if isnan(msg.lat) or isnan(msg.lon):
            return
        self.cur_latlon = LatLon(msg.lat, msg.lon)

        # Update tgt if drone is in TRAVEL or SEARCH state
        threshold = 1  # in metres
        if self.tgt_latlon is not None and self.tgt_latlon.distFromPoint(self.cur_latlon) < threshold:
            if self.drone_state == DroneMode.TRAVEL:
                print(f"Reached {self.tgt_latlon}, currently at {self.cur_latlon}")
                self.drone_state = DroneMode.SEARCH
                print("DRONE: Reached start position, switching to SEARCH state.")
                self.tgt_latlon = self.pathfinder.get_next_waypoint(self.tgt_latlon)
            elif self.drone_state == DroneMode.SEARCH:
                print(f"Reached {self.tgt_latlon}, currently at {self.cur_latlon}")
                self.tgt_latlon = self.pathfinder.get_next_waypoint(self.tgt_latlon)
                
    def fc_recv_vehcmdack(self, msg: VehicleCommandAck):
        # Received VehicleCommandAck from FC
        if msg.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            if msg.result != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                return
            if self.drone_state == DroneMode.INIT_FC:
                # Drone has been armed in offboard control mode, can switch to CONNECT_MC state.
                print("DRONE: Armed in OFFBOARD_CONTROL_MODE.")
                self.drone_state = DroneMode.CONNECT_MC
                self.drone_connect_mc()

    def publish_vehcmdmsg(self, command: int, p1: float = 0.0, p2: float = 0.0, p3: float = 0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.param1 = p1
        msg.param2 = p2
        msg.param3 = p3
        self.pub_vehcom.publish(msg)

    def publish_ready(self):
        msg = Ready()
        msg.drone_id = self.drone_id
        self.pub_mc_ready.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    drone_node = DroneNode()
    
    rclpy.spin(drone_node)

    drone_node.destroy_node()

if __name__ == '__main__':
    main()