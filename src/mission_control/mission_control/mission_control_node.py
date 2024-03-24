# UNCOMMENT FOR PYTHON3.8
# from __future__ import annotations
import logging
import rclpy
import threading
import struct
from queue import Queue, Empty
from rclpy import Future
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from typing import Dict, Tuple, List
from os import environ
from .maplib import LatLon
from .drone_utils import DroneId, DroneState, DroneCommand, DroneMode, DroneCommandId
from mc_interface_msgs.msg import Ready, Detected
from mc_interface_msgs.srv import Command, Status
from .mission_control_webserver import MCWebServer
from .mission_utils import Mission
from .detection_utils import DetectedEntity

logging.basicConfig(encoding='utf-8', level=logging.DEBUG)

COMMAND_CHECK_INTERVAL = 1
RTT_WEIGHTING = 0.125
RTT_TIMEOUT_MULTIPLIER = 10    # This, multiplied by RTT, determines the timeout
MC_HEARTBEAT_INTERVAL = 1      # 1 s

class DroneConnection:
    """
    A connection to a drone. 
    - The drone state must be initialised by the caller, to allow for it to be passed to various threads.
    """
    def __init__(self, drone_id: DroneId, drone_state: DroneState, node: Node):
        self.state = drone_state
        self.node = node
        self.timeout = None
        self.pending_cmd_fut = None
        self.sub_ready = node.create_subscription(
            Ready,
            f"/mc_{drone_id}/mc/out/ready",
            node.mc_recv_ready,
            node.qos_profile,
        )
        self.sub_detected = node.create_subscription(
            Detected,
            f"/mc_{drone_id}/mc/out/detected",
            node.mc_recv_detected,
            node.qos_profile,
        )
        self.cli_cmd = node.create_client(
            Command,
            f"/mc_{drone_id}/mc/srv/cmd",
        )
        self.srv_status = node.create_service(
            Status,
            f"/mc_{drone_id}/mc/srv/status",
            node.mc_recv_status,
        )
    
    def update_rtt(self, sample_rtt: float):
        if self.state._estimated_rtt == 0.0:
            self.state._estimated_rtt = sample_rtt
        else:
            self.estimated_rtt = ((1 - RTT_WEIGHTING) * self.state._estimated_rtt) + (RTT_WEIGHTING * sample_rtt)

        if self.timeout is not None:
            self.node.destroy_timer(self.timeout)

        timeout_interval = MC_HEARTBEAT_INTERVAL + (self.state._estimated_rtt * RTT_TIMEOUT_MULTIPLIER)
        self.timeout = self.node.create_timer(timeout_interval, self.disconnected_action)

    def disconnected_action(self):
        self.node.destroy_timer(self.timeout)
        self.state._mode = DroneMode.DISCONNECTED
        self.state.is_connected = False
        print(f"Warning: Drone {self.state._drone_id} disconnected.")
        
class MCNode(Node):
    def __init__(self, home_pos: LatLon, drone_states: Dict[DroneId, DroneState], commands: Queue[Tuple[DroneId, DroneCommand]], detected_queue: Queue[DetectedEntity]):
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

        # Initialise detectedentity queue
        self.detected_queue: Queue[DetectedEntity] = detected_queue

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
            # Scan for commands from the webserver
            drone_id, command = self.commands.get(block=False)
            if drone_id not in self.connections.keys():
                self.log(f"Warning: No such drone ID {drone_id}")
                return
            self.mc_send_command(drone_id, command)
        except Empty:
            pass

        # Scan all drone connections to check for command responses
        for drone_id, connection in self.connections.items():
            if connection.pending_cmd_fut is not None:
                fut: Future = connection.pending_cmd_fut
                if fut.done():
                    response: Command.Response = fut.result()
                    try:
                        path_b: bytes = b"".join(response.path)
                        pos_ls: List[Tuple[float, float]] = [struct.unpack("!ff", path_b[i:i+8]) for i in range(0, len(path_b), 8)]
                        path: Dict[int, Dict[float, float]] = {}
                        for i, pos in enumerate(pos_ls):
                            path[i] = {"lat": pos[0], "lon": pos[1]}
                        connection.state._set_path(path)
                    except Exception as e:
                        self.log(f"Error when parsing path bytes from drone {drone_id}: {e}")

                    connection.pending_cmd_fut = None

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

    def mc_recv_detected(self, msg: Detected):
        """ MC receives a DETECTED message from drone """
        entity = DetectedEntity.from_message(msg)
        self.detected_queue.put(entity)
        
        self.log(f"Drone {entity.drone_id} reports DETECTED at ({entity.coords.lat}, {entity.coords.lon})")

    def mc_recv_status(self, msg: Status.Request, msg_ack: Status.Response):
        """ MC receives a STATUS update from a drone """
        drone_id = DroneId(msg.drone_id)
        if drone_id not in self.connections.keys():
            self.raise_error(f"Received STATUS message from unknown drone with ID {drone_id}")
            return
        if not self.connections[drone_id].state.is_connected:
            self.log(f"Drone {drone_id} connected.")
            self.connections[drone_id].state.is_connected = True
        
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
    detected_queue: Queue[DetectedEntity] = Queue()

    # Start web server
    mission = Mission()
    webserver = MCWebServer(mission, drone_states, commands, detected_queue)
    webserver_thread = threading.Thread(target=webserver.run, daemon=True)
    webserver_thread.start()

    # Start rclpy node
    rclpy.init(args=args)

    mc_node = MCNode(LatLon(start_lat, start_lon), drone_states, commands, detected_queue)
    rclpy.spin(mc_node)

    mc_node.destroy_node()


if __name__ == '__main__':
    main()