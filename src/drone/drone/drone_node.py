import rclpy
from math import isnan
from enum import IntEnum
from threading import Event
from typing import Tuple, Any
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from .maplib import LatLon, PositionXY
from .pathfinder import PathfinderState
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleGlobalPosition, VehicleLocalPosition, VehicleCommandAck, TrajectorySetpoint
from mc_interface_msgs.msg import Ready, Status
from mc_interface_msgs.srv import Command

CONNFC_CHECK_INTERVAL = 0.5  # Interval to check for the refpoint
CONNFC_TIMEOUT = 15  # Number of seconds until we timeout the refpoint check
IDLE_PING_INTERVAL = 1 #SWITCH TO 30 ONCE SET  # Number of seconds until we ping MC for an update while in IDLE mode
MAX_IDLE_PINGS = 10  # Ping a max of 10 times before we consider it a failure to connect
TIMER_INTERVAL = 0.1  # 100 ms
MC_HEARTBEAT_INTERVAL = 1 # 1 s
DEFAULT_TGT_ALTITUDE = 5

DEFAULT_DRONE_ID = 69  #TODO: fix for multiple drones

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
    - Subscriptions:
        - `sub_vehglobpos`
        - `sub_vehcmdack`
    - Publishers:
        - `pub_trajsp`
        - `pub_vehcom`
        - `pub_ocm`
        - `pub_mc_ready` (MC)
        - `pub_mc_status` (MC)
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
        self.pub_mc_status = None
        self.srv_mc_cmd = None
        self.mc_heartbeat_timer = None

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
        print("DRONE: Drone armed in Offboard Control Mode. Connecting to Mission Control...")
        if self.drone_state != DroneMode.CONNECT_MC:
            raise DroneError(f"Expected CONNECT_MC state, current state is {self.drone_state}")
        
        # Initialise Mission Control communication
        self.pub_mc_ready = self.create_publisher(
            Ready,
            f"/drone_{self.drone_id}/out/ready",
            self.qos_profile,
        )
        self.pub_mc_status = self.create_publisher(
            Status,
            f"/drone_{self.drone_id}/out/status",
            self.qos_profile
        )
        self.srv_mc_cmd = self.create_service(
            Command,
            f"/drone_{self.drone_id}/srv/cmd",
            self.mc_recv_command,
        )

        self.mc_heartbeat_timer = self.create_timer(MC_HEARTBEAT_INTERVAL, self.mc_heartbeat_timer_callback)

        # Switch to IDLE state
        self.drone_state = DroneMode.IDLE
        self.drone_idle()

    def drone_idle(self):
        """ Wait for MC instructions. """
        if self.drone_state != DroneMode.IDLE:
            raise DroneError(f"Expected IDLE state, current state is {self.drone_state}")
        
        #TODO: Send connection message
        self.publish_ready()
        
        # Set timer to wait
        print("DRONE: Connected to MC. Waiting for instructions...")
        self._idle_check_count = 0
        self._idle_timer = self.create_timer(IDLE_PING_INTERVAL, self.drone_idle_ping)
    
    def drone_idle_ping(self):
        """ Ping MC for an update """
        if self.drone_state == DroneMode.IDLE:
            self._idle_check_count += 1
            if self._idle_check_count >= MAX_IDLE_PINGS:
                #TODO: switch to failure mode and RTB
                pass

        #TODO: Send connection ping
        self.publish_ready()
            
        #TODO: if received a command -- this should be shifted to the callback once it's coded
        # self.destroy_timer(self._idle_timer)
        # ## Hardcoded, simulated data for now -- this should come from the MC in the real scenario
        # print("DRONE: Received UPDATE message from MC, switching to TRAVEL state.")
        # sim_update_start_latlon = LatLon(1.340643554050367, 103.9626564184675)
        # sim_update_prob_map = None
        # self.pathfinder = PathfinderState(sim_update_start_latlon, sim_update_prob_map)  # Initialise pathfinding
        # self.tgt_latlon = sim_update_start_latlon
        # print(f"DRONE: Headed to {self.tgt_latlon.toXY(self.ref_latlon)} ({self.tgt_latlon})")

        # self.drone_state = DroneMode.TRAVEL

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
        status = Status()
        status.drone_id = self.drone_id
        status.drone_mode = self.drone_state
        if self.cur_latlon is None:
            status.lat = float('nan')
            status.lon = float('nan')
        else:
            status.lat = self.cur_latlon.lat
            status.lon = self.cur_latlon.lon
        self.pub_mc_status.publish(status)

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
                self.drone_state = DroneMode.CONNECT_MC
                self.drone_connect_mc()

    def mc_recv_command(self, request, response):
        """ Receives a command from MC """
        pass

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