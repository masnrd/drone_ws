"""
drone_utils:
Utilities for interacting with the drones.
"""
import struct
from enum import IntEnum
from rclpy.node import Node
from typing import Union, NewType, Dict
from mc_interface_msgs.srv import Command, Status
from mc_interface_msgs.msg import Ready
from .maplib import LatLon

RTT_WEIGHTING = 0.125
RTT_TIMEOUT_MULTIPLIER = 10    # This, multiplied by RTT, determines the timeout
MC_HEARTBEAT_INTERVAL = 1 # 1 s

# Drone ID type definition
DroneId = NewType("DroneId", int)

# Probability Map type definition: A dictionary, where each H3 hexagon index is mapped to a specific probability.
ProbabilityMap = NewType("ProbabilityMap", Dict[str, float])

class DroneMode(IntEnum):
    """ Current mode reported by the drone """
    DISCONNECTED = -100
    INIT    = 0
    CONN_FC = 1  # Drone is connecting to FC (Flight Controller)
    PREP_FC = 2  # Drone is preparing FC (i.e. arming and taking off)
    CONN_MC = 3  # Drone is connecting to MC (Mission Control)
    IDLE    = 4  # Drone is connected, waiting for instructions
    TRAVEL  = 5  # Drone is moving to a position
    SEARCH  = 6  # Drone is searching a sector
    EXIT    = 7  # Drone is landing and ending its state.

class DroneCommandId(IntEnum):
    RTB           = 0  # Force RTB. No further commands will be accepted.
    SEARCH_SECTOR = 1
    MOVE_TO       = 2  # Go to a specific lat/lon.

class DroneCommand:
    """ Interface for a command """
    def __init__(self, command_id: int, command_data: bytes):
        self.command_id = command_id
        self.command_data = command_data

    def generate_command(self, drone_id: DroneId) -> Command.Request:
        """ Generates a ROS2 Request that will be sent over ROS to the drone. """
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
    def __init__(self, sector_start: LatLon, sector_prob_map: ProbabilityMap):
        #TODO: encode prob_map
        command_data = struct.pack("!ff", sector_start.lat, sector_start.lon)
        super().__init__(DroneCommandId.SEARCH_SECTOR, command_data)

class DroneCommand_MOVE_TO(DroneCommand):
    def __init__(self, tgt_pos: LatLon):
        command_data = struct.pack("!ff", tgt_pos.lat, tgt_pos.lon)
        super().__init__(DroneCommandId.MOVE_TO, command_data)


class DroneState:
    """
    State of the drone.
    - The webserver using this SHOULD NOT MODIFY any of the attributes, and should instead
    access data via the getter methods provided.
    - This is because attribute modification is done by the ROS2 node which runs in a separate thread.
    """
    def __init__(self, drone_id: DroneId):
        self._drone_id = drone_id
        self._mode = DroneMode.DISCONNECTED
        self._battery_percentage = 0.0
        self._estimated_rtt = 0.0
        self._position: Union[LatLon, None] = None
        self._last_command: Union[DroneCommand, None] = None

    def get_drone_id(self) -> DroneId:
        return self._drone_id
    
    def get_mode(self) -> DroneMode:
        return self._mode
    
    def get_battery_percentage(self) -> float:
        return self._battery_percentage
    
    def get_estimated_rtt(self) -> float:
        return self._estimated_rtt
    
    def get_position(self) -> Union[LatLon, None]:
        """ Returns the current LatLon of the drone, or None if not set yet. """
        return self._position
    
    def get_last_command(self) -> Union[DroneCommand, None]:
        """ Returns the most recent DroneCommand sent to the drone, or None if not set yet. """
        return self._last_command

    def toJSON(self) -> str:
        lat, lon = "null", "null"
        if self._position is not None:
            lat, lon = self._position.lat, self._position.lon
        command = "-"
        if self._last_command is not None:
            command = DroneCommandId(self._last_command.command_id).name
        
        ret = "{"
        ret += f"\"drone_id\": {self._drone_id},"
        ret += f"\"mode\": \"{DroneMode(self._mode).name}\", "
        ret += f"\"battery_percentage\": {self._battery_percentage}, "
        ret += f"\"estimated_rtt\": {self._estimated_rtt}, "
        ret += f"\"lat\": {lat}, "
        ret += f"\"lon\": {lon}, "
        ret += f"\"last_command\": \"{command}\" "
        ret += "}"

        return ret
    
    def __repr__(self) -> str:
        lat, lon = float('nan'), float('nan')
        if self._position is not None:
            lat, lon = self._position.lat, self._position.lon
        
        return f"Drone {self._drone_id}:\n\tPosition: {lat}, {lon}\n\tMode: {DroneMode(self._mode).name}\n\tBattery: {self._battery_percentage}%\n\tRTT: {self._estimated_rtt}"


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
        print(f"Warning: Drone {self.state._drone_id} disconnected.")