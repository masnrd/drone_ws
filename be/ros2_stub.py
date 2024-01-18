"""
ros2_stub:
Classes and functions to replace ROS2 dependencies.
"""
from enum import IntEnum
from typing import NewType

# Drone ID type definition
DroneId = NewType("DroneId", int)

class DroneCommandId(IntEnum):
    RTB           = 0  # Force RTB. No further commands will be accepted.
    SEARCH_SECTOR = 1
    MOVE_TO       = 2  # Go to a specific lat/lon.

class DroneMode(IntEnum):
    """ Current mode reported by the drone """
    ERROR        = -99
    RTB          = -2  # Drone is returning to MC
    IDLE         = -1  # Drone is idle
    INIT         = 0   # Complete initialisation
    CONNECT_FC   = 1   # Connecting to Flight Controller (FC)
    INIT_FC      = 2   # Initialisation of subscriptions and publishers
    CONNECT_MC   = 3   # Connecting to Mission Control (MC)
    TRAVEL       = 4   # Moving to Sector
    SEARCH       = 5   # Searching Sector

class DroneMessage:
    def __init__(self):
        pass

class Ready(DroneMessage):
    """ (ROS2 stub) Represents a ready message sent from the drone, to indicate that it's ready for commands. """
    def __init__(self, drone_id: DroneId):
        self.drone_id = drone_id

class Command:
    """ (ROS2 stub) Represents a command being sent from mission control to the drone """
    class Request(DroneMessage):
        """ Sent from MC """
        def __init__(self, drone_id: DroneId = -1,  command_id: DroneCommandId = -1, command_bytes: bytes = b""):
            self.drone_id = drone_id
            self.cmd_id = command_id
            self.cmd_data = command_bytes
    class Response(DroneMessage):
        """ Sent from drone """
        def __init__(self, drone_id: DroneId = -1, command_id: DroneCommandId = -1):
            self.drone_id = drone_id
            self.cmd_id = command_id

class Status:
    """ 
    (ROS2 stub) Represents a status update sent from the drone to mission control
    - Note: `timestamp` is in MICROSECONDS.
    - Note: `last_rtt` is the RTT of the previous request-response exchange from the drone's perspective, and is used by mission control to estimate RTT.
    """
    class Request(DroneMessage):
        """ Sent from drone """
        def __init__(self, timestamp: int = 0, drone_id: DroneId = -1, drone_mode: DroneMode = 0, lat: float = 0.0, lon: float = 0.0, battery_percentage: float = 0.0, battery_secondsleft: float = 0.0, last_rtt: float = 0.0):
            self.timestamp = timestamp
            self.drone_id, self.drone_mode = drone_id, drone_mode
            self.lat, self.lon = lat, lon
            self.battery_percentage, self.battery_secondsleft = battery_percentage, battery_secondsleft
            self.last_rtt = last_rtt
    class Response(DroneMessage):
        """ 
        Sent from MC
        - Note: `status_timestamp` is the timestamp of the request this is for. This is used by the drone to compute RTT.
        """
        def __init__(self, status_timestamp: int = 0, drone_id: DroneId = -1, last_command_id: DroneCommandId = -1):
            self.status_timestamp = status_timestamp
            self.drone_id = drone_id
            self.last_cmd_id = last_command_id
