import rclpy
from math import isnan
from enum import IntEnum
from threading import Event
from typing import Tuple, Any
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from .maplib import LatLon, PositionXY
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleGlobalPosition, VehicleLocalPosition, VehicleCommandAck, TrajectorySetpoint

CONNFC_CHECK_INTERVAL = 0.5  # Interval to check for the refpoint
CONNFC_TIMEOUT = 15  # Number of seconds until we timeout the refpoint check
TIMER_INTERVAL = 0.1  # 100 ms

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
    - Subscriptions:
        - `sub_vehglobpos`
        - `sub_vehcmdack`
    - Publishers:
        - `pub_trajsp`
        - `pub_vehcom`
        - `pub_ocm`
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

        self.drone_init()

    def drone_init(self):
        """ Drone initialisation """
        print("DRONE: Initialising.")
        if self.drone_state != DroneMode.INIT:
            raise DroneError(f"Expected INIT state, current state is {self.drone_state}")
        
        self.ref_latlon = None
        self.sub_vehglobpos = None
        self.sub_vehcmdack = None
        self.pub_trajsp = None
        self.pub_vehcom = None
        self.pub_ocm = None
        self.heartbeat_timer = None
        self._counter = None

        # Start subscriber for reference point
        self.ref_latlon = None
        self._connfc_check_count = 0
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

        if self.ref_latlon is None:
            self._connfc_check_count += 1
            return

        # Here, we've established the reference point and can proceed to the INIT_FC stage.
        self.destroy_timer(self._connfc_timer)
        self.destroy_subscription(self._connfc_sub_vehlocpos)
        print(f"DRONE: Connected to FC. Reference LatLon: {self.ref_latlon}")
        self.drone_state = DroneMode.INIT_FC
        self.drone_init_fc()

    def _fc_recv_locpos(self, msg: VehicleLocalPosition):
        if isnan(msg.ref_lat) or isnan(msg.ref_lon):
            return
        self.ref_latlon = LatLon(msg.ref_lat, msg.ref_lon)

    def drone_init_fc(self):
        """ Initialise the drone state and connect to flight controller """
        print("DRONE: Initialising FC.")
        if self.drone_state != DroneMode.INIT_FC:
            raise DroneError(f"Expected INIT_FC state, current state is {self.drone_state}")

        self.sub_vehglobpos = self.create_subscription(
            VehicleGlobalPosition,
            "/fmu/out/vehicle_global_position",
            self.fc_recv_vehglobpos,
            self.qos_profile,
        )
        self.sub_vehcmdack = self.create_subscription(
            VehicleCommandAck,
            "/fmu/out/vehicle_command_ack",
            self.fc_recv_vehcmdack,
            self.qos_profile,
        )
        self.pub_trajsp = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            10,
        )
        self.pub_vehcom = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            10,
        )
        self.pub_ocm = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            10,
        )

        # Attempt to arm drone in Offboard mode
        self._counter = 0
        self.heartbeat_timer = self.create_timer(TIMER_INTERVAL, self.heartbeat_timer_callback)

    def drone_connect_mc(self):
        """ Establish a connection to Mission Control """
        print("DRONE: Drone armed in Offboard Control Mode. Connecting to Mission Control...")
        if self.drone_state != DroneMode.CONNECT_MC:
            raise DroneError(f"Expected CONNECT_MC state, current state is {self.drone_state}")

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
        
        # Publish heartbeat
        hb_msg = OffboardControlMode()
        hb_msg.position = True
        hb_msg.velocity = False
        hb_msg.acceleration = False
        hb_msg.body_rate = False
        hb_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.pub_ocm.publish(hb_msg)

    def fc_recv_vehglobpos(self, msg: VehicleGlobalPosition):
        # Received VehicleGlobalPosition from FC
        pass

    def fc_recv_vehcmdack(self, msg: VehicleCommandAck):
        # Received VehicleCommandAck from FC
        if msg.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            if msg.result != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                return
            if self.drone_state == DroneMode.INIT_FC:
                # Drone has been armed in offboard control mode, can switch to CONNECT_MC state.
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

def main(args=None):
    rclpy.init(args=args)
    drone_node = DroneNode()
    
    rclpy.spin(drone_node)

    drone_node.destroy_node()

if __name__ == '__main__':
    main()