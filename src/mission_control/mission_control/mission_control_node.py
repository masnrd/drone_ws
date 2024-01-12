import rclpy
import struct
import threading
import json
from queue import Queue, Empty
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from enum import IntEnum
from typing import Dict, Tuple
from flask import Flask, request
from .maplib import LatLon
from .drone_utils import DroneId, DroneConnection, DroneState, DroneCommand, DroneMode, DroneCommandId
from .drone_utils import DroneCommand_RTB, DroneCommand_MOVE_TO, DroneCommand_SEARCH_SECTOR
from mc_interface_msgs.msg import Ready
from mc_interface_msgs.srv import Command, Status

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
        print("MISSION CONTROL: Initialised.")

    def log(self, msg: str):
        self.get_logger().info(f"MISSION CONTROL: {msg}")

    def raise_error(self, msg: str):
        self.get_logger().error(f"MISSION CONTROL ERROR: {msg}")
        raise Exception(msg)
    
    def check_command_loop(self):
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

        connection.state.last_command = command
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
        state.position = LatLon(msg.lat, msg.lon)
        state.mode = DroneMode(msg.drone_mode)
        state.battery_percentage = msg.battery_percentage

        # Send Response
        msg_ack.status_timestamp = msg.timestamp
        msg_ack.drone_id = drone_id

        if state.last_command is not None:
            msg_ack.last_cmd_id = state.last_command.cmd_id
        else:
            msg_ack.last_cmd_id = -1

        return msg_ack

class MCWebServer:
    def __init__(self, drones: Dict[int, DroneState], commands: Queue[Tuple[DroneId, DroneCommand]]):
        self.app = Flask("Mission Control")
        self.drones = drones
        self.commands: Queue[Tuple[DroneId, DroneCommand]] = commands

        # Set up Endpoints
        self.app.add_url_rule("/api/info", view_func=self.route_info)
        self.app.add_url_rule("/api/action/search", view_func=self.route_action_search)

    def route_info(self) -> Dict[int, str]:
        ret: Dict[int, str] = {}
        for drone_id, drone in self.drones.items():
            ret[drone_id] = drone.toJSON()
        
        return ret
    
    def route_action_search(self) -> Dict[int, str]:
        drone_id = request.args.get("drone_id", type=int, default=None)
        lat = request.args.get("lat", type=float, default=None)
        lon = request.args.get("lon", type=float, default=None)

        if drone_id is None:
            return {"error": "need drone_id"}, 400

        if lat is None or lon is None:
            return {"error": "need latitude/longitude parameters as float"}, 400
        
        command_tup = (drone_id, DroneCommand_SEARCH_SECTOR(LatLon(lat, lon), None))

        self.commands.put_nowait(command_tup)
        
        return {}, 200
    
    def run(self):
        self.app.run(debug=True, use_reloader=False)

def main(args=None):
    drone_states = {
        DroneId(69): DroneState(69)
    }
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