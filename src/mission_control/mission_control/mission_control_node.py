import rclpy
import struct
import threading
import json
from queue import Queue
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from enum import IntEnum
from typing import Dict
from flask import Flask
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
    ERROR      = -99
    UNKNOWN    = -3  # Drone has not reported yet
    RTB        = -2  # Drone is returning to MC
    IDLE       = -1  # Drone is idle
    INIT       = 0   # Complete initialisation
    CONNECT_FC = 1   # Connecting to Flight Controller (FC)
    INIT_FC    = 2   # Initialisation of subscriptions and publishers
    CONNECT_MC = 3   # Connecting to Mission Control (MC)
    TRAVEL     = 4   # Moving to Sector
    SEARCH     = 5   # Searching Sector

class DroneCommandId(IntEnum):
    RTB           = 0  # Force RTB. No further commands will be accepted.
    SEARCH_SECTOR = 1
    MOVE_TO       = 2  # Go to a specific lat/lon.

class DroneCommand:
    """ Interface for a command """
    def __init__(self, command_id: int, command_data: bytes):
        self.command_id = command_id
        self.command_data = command_data

    def generate_command(self, drone_id: int) -> Command.Request:
        cmd = Command.Request()
        cmd.drone_id = drone_id
        cmd.cmd_id = self.command_id
        cmd.cmd_data = self.command_data
        return cmd

class DroneCommand_RTB(DroneCommand):
    def __init__(self, base_pos: LatLon):
        command_data = struct.pack("!ff", base_pos.lat, base_pos.lon)
        super().__init__(DroneCommandId.RTB, command_data)

class DroneCommand_SEARCH_SECTOR(DroneCommand):
    def __init__(self, sector_start: LatLon, sector_prob_map):
        #TODO: encode prob_map
        command_data = struct.pack("!ff", sector_start.lat, sector_start.lon)
        super().__init__(DroneCommandId.SEARCH_SECTOR, command_data)

class DroneCommand_MOVE_TO(DroneCommand):
    def __init__(self, tgt_pos: LatLon):
        command_data = struct.pack("!ff", tgt_pos.lat, tgt_pos.lon)
        super().__init__(DroneCommandId.MOVE_TO, command_data)

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
        self.pending_command_fut = None
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

    def toJSON(self) -> str:
        return json.dumps(self, cls=DroneStateEncoder)

class DroneStateEncoder(json.JSONEncoder):
    def default(self, o: DroneState):

        
        return {
            "drone_id": o.drone_id,
            "mode": DroneMode(o.mode).name,
            "reported_mode": ReportedDroneMode(o.reported_mode).name,
            "reported_battery_percentage": o.reported_battery_percentage,
            "estimated_rtt": o.estimated_rtt,
            "lat": o.position.lat if o.position is not None else float('nan'),
            "lon": o.position.lon if o.position is not None else float('nan'),
            "last_command": DroneCommandId(o.last_command.cmd_id).name if o.last_command is not None else None,
        }
    

class MCNode(Node):
    def __init__(self, drones_dict: Dict[int, DroneState], commands: Queue):
        super().__init__("mission_control")
        self.drones:Dict[int, DroneState] = drones_dict
        self.commands = commands
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.get_logger().info("Mission Control initialised.")

        self.add_drone(69)  #TODO: Handle multiple drones
        print("MISSION CONTROL: Initialised.")

    def add_drone(self, drone_id: int):
        self.drones[drone_id] = DroneState(drone_id, self)

    def mc_send_command(self, drone_cmd: DroneCommand, drone_id: int):
        drone = self.drones[drone_id]
        command = drone_cmd.generate_command(drone_id)
        drone.last_command = command
        drone.pending_command_fut = drone.cli_cmd.call_async(command)

    def mc_recv_ready(self, msg: Ready):
        """ MC receives a READY message """
        # 1. Identify drone by ID
        drone_id = msg.drone_id

        # 2. Update drone status
        self.drones[drone_id].mode = DroneMode.READY
        self.get_logger().info(f"MISSION CONTROL: Drone {drone_id} reports READY")

        #TODO: temporary hardcoded command sent
        self.hardcoded_send_command(drone_id)

    def mc_recv_status(self, msg: Status.Request, msg_ack: Status.Response):
        """ MC receives a STATUS update from a drone """
        # 1. Identify drone by ID
        drone_id = msg.drone_id
        drone = self.drones[drone_id]

        # 2. Update local record of drone position and drone state
        drone.position = LatLon(msg.lat, msg.lon)
        drone.reported_mode = ReportedDroneMode(msg.drone_mode)
        drone.reported_battery_percentage = msg.battery_percentage
        drone.update_rtt(msg.last_rtt)
        self.get_logger().info(f"MISSION CONTROL: {drone}")

        # 3. Send response
        msg_ack.status_timestamp = msg.timestamp
        msg_ack.drone_id = drone_id

        if drone.last_command is not None:
            msg_ack.last_cmd_id = drone.last_command.cmd_id
        else:
            msg_ack.last_cmd_id = -1

        return msg_ack

    def hardcoded_send_command(self, drone_id: int):
        tgt = LatLon(1.340643554050367, 103.9626564184675)
        drone_cmd = DroneCommand_SEARCH_SECTOR(tgt, None)
        self.mc_send_command(drone_cmd, drone_id)

class MCWebServer:
    def __init__(self, drones: Dict[int, DroneState], commands: Queue):
        self.app = Flask("Mission Control")
        self.drones = drones
        self.commands = commands

        # Set up Endpoints
        self.app.add_url_rule("/", view_func=self.route_index)

    def route_index(self) -> Dict[int, str]:
        ret: Dict[int, str] = {}
        for drone_id, drone in self.drones.items():
            ret[drone_id] = drone.toJSON()
        
        return ret
    
    def run(self):
        self.app.run(debug=True, use_reloader=False)

def main(args=None):
    drones = {}
    commands = Queue()

    # Start web server
    webserver = MCWebServer(drones, commands)
    webserver_thread = threading.Thread(target=webserver.run, daemon=True)
    webserver_thread.start()

    # Start rclpy node
    rclpy.init(args=args)

    mc_node = MCNode(drones, commands)
    rclpy.spin(mc_node)

    mc_node.destroy_node()

if __name__ == '__main__':
    main()