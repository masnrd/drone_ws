"""
drone_utils:
Utilities for interacting with the drones.
"""
import struct
from enum import IntEnum
from rclpy.node import Node
from typing import Union, NewType, Dict, Any
from mc_interface_msgs.srv import Command, Status
from mc_interface_msgs.msg import Ready
from .maplib import LatLon

# Drone ID type definition
DroneId = NewType("DroneId", int)

# Probability Map type definition: A dictionary, where each H3 hexagon index is mapped to a specific probability.
ProbabilityMap = NewType("ProbabilityMap", Dict[str, float])

class DroneMode(IntEnum):
    """ Current mode reported by the drone """
    DISCONNECTED = -100
    INIT    = 0
    CONN_FC = 1  # Drone is connecting to FC (Flight Controller)
    CONN_MC = 2  # Drone is connecting to MC (Mission Control)
    TAKEOFF = 3  # Drone is taking off
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

    def get_dict(self) -> Dict[str, Any]:
        """ Get the dictionary of the drone's values, which can be JSONified. """
        lat, lon = None, None
        if self._position is not None:
            lat, lon = self._position.lat, self._position.lon
        command = "-"
        if self._last_command is not None:
            command = DroneCommandId(self._last_command.command_id).name

        ret = {
            "drone_id": str(self.get_drone_id()),
            "mode": str(self.get_mode().name),
            "battery_percentage": self.get_battery_percentage(),
            "estimated_rtt": self.get_estimated_rtt(),
            "position": {
                "lat": lat, "lon": lon
            },
            "last_command": command,
        }
        
        return ret
    
    def __repr__(self) -> str:
        lat, lon = float('nan'), float('nan')
        if self._position is not None:
            lat, lon = self._position.lat, self._position.lon
        
        return f"Drone {self._drone_id}:\n\tPosition: {lat}, {lon}\n\tMode: {DroneMode(self._mode).name}\n\tBattery: {self._battery_percentage}%\n\tRTT: {self._estimated_rtt}"
