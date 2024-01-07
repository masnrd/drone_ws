import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from enum import IntEnum
from typing import Dict
from .maplib import LatLon
from mc_interface_msgs.msg import Ready, Status
from mc_interface_msgs.srv import Command

class DroneMode(IntEnum):
    """ Drone mode (from perspective of Mission Control) """
    DISCONNECTED = 0
    READY        = 1  # Received READY from drone
    IN_OPERATION = 2  # Received CMD_ACK from drone after sending CMD

class ReportedDroneMode(IntEnum):
    """ Drone mode reported by the drone itself """
    UNKNOWN    = -3  # Drone has not reported yet
    RTB        = -2  # Drone is returning to MC
    IDLE       = -1  # Drone is idle
    INIT       = 0   # Complete initialisation
    CONNECT_FC = 1   # Connecting to Flight Controller (FC)
    INIT_FC    = 2   # Initialisation of subscriptions and publishers
    CONNECT_MC = 3   # Connecting to Mission Control (MC)
    TRAVEL     = 4   # Moving to Sector
    SEARCH     = 5   # Searching Sector

class DroneState:
    def __init__(self, drone_id: int, node: 'MCNode'):
        self.drone_id = drone_id
        self.mode = DroneMode.DISCONNECTED
        self.reported_mode = ReportedDroneMode.UNKNOWN
        self.reported_battery_percentage = 0.0
        self.position = None
        self.last_command = None

        self.sub_ready = node.create_subscription(
            Ready,
            f"/drone_{drone_id}/out/ready",
            node.mc_recv_ready,
            node.qos_profile,
        )
        self.sub_status = node.create_subscription(
            Status,
            f"/drone_{drone_id}/out/status",
            node.mc_recv_status,
            node.qos_profile,
        )
        self.cmd_cli = node.create_client(
            Command,
            f"/drone_{drone_id}/srv/cmd",
        )

class MCNode(Node):
    def __init__(self):
        super().__init__("mission_control")
        self.drones:Dict[int, DroneState] = {}
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.add_drone(69)  #TODO: Handle multiple drones
        print("MISSION CONTROL: Initialised.")

    def add_drone(self, drone_id: int):
        self.drones[drone_id] = DroneState(drone_id, self)

    def mc_send_command(self, command_id: int, drone_id: int):
        command = Command.Request()
        drone = self.drones[drone_id]
        #TODO: Send request and handle response

    def mc_recv_ready(self, msg: Ready):
        """ MC receives a READY message """
        # 1. Identify drone by ID
        drone_id = msg.drone_id

        # 2. Update drone status
        self.drones[drone_id].mode = DroneMode.READY
        print(f"MISSION CONTROL: Drone {drone_id} reports READY")

    def mc_recv_status(self, msg: Status):
        """ MC receives a STATUS update from a drone """
        # 1. Identify drone by ID
        drone_id = msg.drone_id

        # 2. Update local record of drone position and drone state
        self.drones[drone_id].position = LatLon(msg.lat, msg.lon)
        self.drones[drone_id].reported_mode = ReportedDroneMode(msg.drone_mode)
        self.drones[drone_id].reported_battery_percentage = msg.battery_percentage
        print(f"MISSION CONTROL: Drone {drone_id}:\n\tPosition: {self.drones[drone_id].position}\n\tMode: {self.drones[drone_id].reported_mode}\n\tBattery: {self.drones[drone_id].reported_battery_percentage}%.")


def main(args=None):
    rclpy.init(args=args)
    mc_node = MCNode()
    
    rclpy.spin(mc_node)

    mc_node.destroy_node()

if __name__ == '__main__':
    main()