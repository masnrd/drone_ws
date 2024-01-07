import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from enum import IntEnum
from typing import Dict
from .maplib import LatLon
from mc_interface_msgs.msg import Ready
from mc_interface_msgs.srv import Command, Status

RTT_WEIGHTING = 0.125
INITIAL_RTT_ESTIMATE = 0.0015 # for simulation
RTT_TIMEOUT_MULTIPLIER = 10    # This, multiplied by RTT, determines the timeout
MC_HEARTBEAT_INTERVAL = 1 # 1 s

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
        self.node = node
        self.mode = DroneMode.DISCONNECTED
        self.reported_mode = ReportedDroneMode.UNKNOWN
        self.reported_battery_percentage = 0.0
        self.estimated_rtt = INITIAL_RTT_ESTIMATE
        self.position = None
        self.last_command = None
        self.timeout = None

        self.sub_ready = node.create_subscription(
            Ready,
            f"/drone_{drone_id}/out/ready",
            node.mc_recv_ready,
            node.qos_profile,
        )
        self.cli_cmd = node.create_client(
            Command,
            f"/drone_{drone_id}/srv/cmd",
        )
        self.srv_status = node.create_service(
            Status,
            f"/drone_{drone_id}/srv/status",
            node.mc_recv_status,
        )

    def update_rtt(self, sample_rtt: float):
        self.estimated_rtt = ((1 - RTT_WEIGHTING) * self.estimated_rtt) + (RTT_WEIGHTING * sample_rtt)
        if self.timeout is not None:
            self.node.destroy_timer(self.timeout)

        timeout_interval = MC_HEARTBEAT_INTERVAL + (self.estimated_rtt * RTT_TIMEOUT_MULTIPLIER)
        self.timeout = self.node.create_timer(timeout_interval, self.disconnected)

    def disconnected(self):
        self.node.destroy_timer(self.timeout)
        self.mode = DroneMode.DISCONNECTED
        print(f"Warning: Drone {self.drone_id} disconnected.")

    def __repr__(self) -> str:
        return f"Drone {self.drone_id}:\n\tPosition: {self.position}\n\tMode: {ReportedDroneMode(self.reported_mode).name}\n\tBattery: {self.reported_battery_percentage}%\n\tRTT: {self.estimated_rtt}"


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

    def mc_recv_status(self, msg: Status.Request, msg_ack: Status.Response):
        """ MC receives a STATUS update from a drone """
        # 1. Identify drone by ID
        drone_id = msg.drone_id

        # 2. Update local record of drone position and drone state
        self.drones[drone_id].position = LatLon(msg.lat, msg.lon)
        self.drones[drone_id].reported_mode = ReportedDroneMode(msg.drone_mode)
        self.drones[drone_id].reported_battery_percentage = msg.battery_percentage
        self.drones[drone_id].update_rtt(msg.last_rtt)
        print(f"MISSION CONTROL: {self.drones[drone_id]}")

        # 3. Send response
        msg_ack.status_timestamp = msg.timestamp
        msg_ack.drone_id = drone_id
        msg_ack.last_cmd_id = -1

        return msg_ack

def main(args=None):
    rclpy.init(args=args)
    mc_node = MCNode()
    
    rclpy.spin(mc_node)

    mc_node.destroy_node()

if __name__ == '__main__':
    main()