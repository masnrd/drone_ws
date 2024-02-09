# UNCOMMENT FOR PYTHON3.8
# from __future__ import annotations
import rclpy
import threading
from queue import Queue, Empty
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from typing import Dict, Tuple
from os import environ
from .maplib import LatLon
from .drone_utils import DroneId, DroneConnection, DroneState, DroneCommand, DroneMode, DroneCommandId
from mc_interface_msgs.msg import Ready
from mc_interface_msgs.srv import Command, Status
from .mission_control_webserver import MCWebServer

COMMAND_CHECK_INTERVAL = 1

class MCNode(Node):
    def __init__(self, drone_states: Dict[DroneId, DroneState], commands: Queue[Tuple[DroneId, DroneCommand]]):
        super().__init__("mission_control")

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Initialise command queue
        self.commands: Queue[Tuple[DroneId, DroneCommand]] = commands
        self.command_loop = self.create_timer(COMMAND_CHECK_INTERVAL, self.check_command_loop)

        # Initialise drone connections
        self.connections:Dict[DroneId, DroneConnection] = {}
        for drone_id, drone_state in drone_states.items():
            self.connections[drone_id] = DroneConnection(drone_id, drone_state, self)
        
        self.get_logger().info("Mission Control initialised.")

    def log(self, msg: str):
        self.get_logger().info(f"MISSION CONTROL: {msg}")

    def raise_error(self, msg: str):
        self.get_logger().error(f"MISSION CONTROL ERROR: {msg}")
        raise Exception(msg)
    
    def check_command_loop(self):
        """ Loop to check for commands from the webserver """
        try:
            drone_id, command = self.commands.get(block=False)
            print(f"MISSION CONTROL: User entered command {DroneCommandId(command.command_id).name}")
            self.mc_send_command(drone_id, command)
        except Empty:
            pass

    def mc_send_command(self, drone_id: DroneId, drone_cmd: DroneCommand):
        """ MC sends a Command to a given drone """
        connection = self.connections[drone_id]
        command = drone_cmd.generate_command(drone_id)

        connection.state._last_command = drone_cmd
        connection.pending_cmd_fut = connection.cli_cmd.call_async(command)

    def mc_recv_ready(self, msg: Ready):
        """ MC receives a READY message from drone """
        drone_id = DroneId(msg.drone_id)
        if drone_id not in self.connections.keys():
            self.raise_error(f"Received READY message from unknown drone with ID {drone_id}")
            return
        
        self.log(f"Drone {drone_id} reports READY")

    def mc_recv_status(self, msg: Status.Request, msg_ack: Status.Response):
        """ MC receives a STATUS update from a drone """
        drone_id = DroneId(msg.drone_id)
        if drone_id not in self.connections.keys():
            self.raise_error(f"Received STATUS message from unknown drone with ID {drone_id}")
            return
        
        # Update state
        self.connections[drone_id].update_rtt(msg.last_rtt)
        state = self.connections[drone_id].state
        state._position = LatLon(msg.lat, msg.lon)
        state._mode = DroneMode(msg.drone_mode)
        state._battery_percentage = msg.battery_percentage

        # Send Response
        msg_ack.status_timestamp = msg.timestamp
        msg_ack.drone_id = drone_id

        if state._last_command is not None:
            msg_ack.last_cmd_id = state._last_command.command_id
        else:
            msg_ack.last_cmd_id = -1

        return msg_ack

def main(args=None):
    # Load env vars
    drone_count = int(environ.get("SIM_DRONE_COUNT", "2"))
    start_lat, start_lon = float(environ.get("PX4_HOME_LAT", 0.0)), float(environ.get("PX4_HOME_LON", 0.0))
    
    # Generate initial state
    print(f"Using Start Location: ({start_lat}, {start_lon})")
    print(f"Drone Count: {drone_count}")
    drone_states = {}
    for drone_id in range(1, drone_count+1):
        drone_states[DroneId(drone_id)] = DroneState(drone_id)
    commands: Queue[Tuple[DroneId, DroneCommand]] = Queue()

    # Start web server
    webserver = MCWebServer(drone_states, commands)
    webserver_thread = threading.Thread(target=webserver.run, daemon=True)
    webserver_thread.start()

    # Start rclpy node
    rclpy.init(args=args)

    mc_node = MCNode(drone_states, commands)
    rclpy.spin(mc_node)

    mc_node.destroy_node()

if __name__ == '__main__':
    main()