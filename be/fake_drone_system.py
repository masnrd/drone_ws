"""
fake_drone_system:
A fake system of drones to simulate interaction between the ROS2 mission control node and drone nodes
"""
import struct
import random
import logging
from threading import Lock, Event, RLock, Thread
from typing import Dict, Tuple, Any
from queue import Queue
from time import sleep

from drone_utils import DroneId, DroneState, DroneMode, DroneCommand, DroneCommandId
from maplib import LatLon
from pathfinder import PathfinderState

DRONE_CYCLE_INTERVAL = 1
DRONE_SPEED = 3  # In metres/s, UNTESTED.

def unpack_command(cmd: DroneCommand) -> Dict[str, Any]:
    """ Unpacks a Command from the Mission Control. """
    cmd_id = cmd.command_id
    cmd_data = cmd.command_data #b''.join(cmd.command_data)

    ret = {"command_id": cmd_id}
    if cmd_id == DroneCommandId.RTB:
        coords = struct.unpack("!ff", cmd_data)
        ret["base_pos"] = LatLon(coords[0], coords[1])
    elif cmd_id == DroneCommandId.SEARCH_SECTOR:
        coords = struct.unpack("!ff", cmd_data)
        ret["sector_start_pos"] = LatLon(coords[0], coords[1])
        ret["sector_prob_map"] = None  #TODO: decide how to encode prob_map for MC
    elif cmd_id == DroneCommandId.MOVE_TO:
        coords = struct.unpack("!ff", cmd_data)
        ret["goto_pos"] = LatLon(coords[0], coords[1])
    
    return ret

class _SingletonMeta(type):
    _instances = {}
    _lock = Lock()
    def __call__(cls, *args, **kwargs):
        with cls._lock:
            if cls not in cls._instances:
                cls._instances[cls] = super().__call__(*args, **kwargs)
        return cls._instances[cls]
    
class Drone:
    """ Fake drone """
    def __init__(self, drone_id: DroneId, drone_states: Dict[DroneId, DroneState], start_pos: LatLon):
        self.drone_id = drone_id
        self.logger = logging.getLogger(f"drone_{drone_id}")
        self.drone_states = drone_states
        self.drone_states[drone_id]._position = start_pos
        self.target_pos = start_pos
        self.pathfinder = None
        self.next_mode = None
        self.speed = DRONE_SPEED
        self._lock = RLock()

    def log(self, msg: str):
        self.logger.info(msg)

    def cycle(self):
        match self.drone_states[self.drone_id].get_mode():
            case DroneMode.INIT | DroneMode.CONNECT_FC | DroneMode.INIT_FC | DroneMode.CONNECT_MC:
                self.set_drone_mode(DroneMode.IDLE)
            case DroneMode.IDLE:
                pass
            case DroneMode.TRAVEL:
                self.updatePositionWithCycle()
            case DroneMode.SEARCH:
                self.updatePositionWithCycle()
            case DroneMode.RTB:
                self.updatePositionWithCycle()
            case DroneMode.ERROR:
                pass
            case DroneMode.DISCONNECTED:
                pass

    def updatePositionWithCycle(self):
        """ Update the position of the drone in a single cycle """
        threshold = 1
        with self._lock:
            pos = self.drone_states[self.drone_id].get_position()
            dist = self.target_pos.distFromPoint(pos)
            if pos is None:
                return
            
            if dist <= threshold:
                match self.drone_states[self.drone_id].get_mode():
                    case DroneMode.TRAVEL:
                        if self.next_mode is None:
                            raise NotImplementedError()
                        self.set_drone_mode(self.next_mode)
                    case DroneMode.SEARCH:
                        if self.pathfinder is None:
                            raise NotImplementedError(f"Drone {self.drone_id} in SEARCH mode, but no pathfinder object found")
                        
                        self.target_pos = self.pathfinder.get_next_waypoint(self.target_pos)
                    case DroneMode.RTB:
                        self.set_drone_mode(self.next_mode)
                return
            
            # Otherwise, we move by a certain speed
            if dist >= self.speed:
                # Here, we won't reach the target within the next cycle.
                distance_vec = pos.toXY(self.target_pos)
                distance_vec = self.target_pos.toXY(pos)
                distance_vec.x = (distance_vec.x / dist) * self.speed
                distance_vec.y = (distance_vec.y / dist) * self.speed

                self.drone_states[self.drone_id]._position = distance_vec.toLatLon()
            else:
                # Here, we WILL reach the target within the next cycle, but speed will overshoot.
                distance_vec = pos.toXY(self.target_pos)
                distance_vec = self.target_pos.toXY(pos)
                distance_vec.x = (distance_vec.x / dist) * dist
                distance_vec.y = (distance_vec.y / dist) * dist

                self.drone_states[self.drone_id]._position = distance_vec.toLatLon()

    def handle_command(self, drone_command: DroneCommand):
        print(f"Drone {self.drone_id}: Received command {DroneCommandId(drone_command.command_id).name}")
        command = unpack_command(drone_command)
        match drone_command.command_id:
            case DroneCommandId.RTB:
                self.set_drone_mode(DroneMode.TRAVEL, DroneMode.IDLE)
                self.set_drone_target_pos(command["base_pos"].lat, command["base_pos"].lon)
            case DroneCommandId.MOVE_TO:
                self.set_drone_mode(DroneMode.TRAVEL, DroneMode.IDLE)
                self.set_drone_target_pos(command["goto_pos"].lat, command["goto_pos"].lon)
            case DroneCommandId.SEARCH_SECTOR:
                self.set_drone_mode(DroneMode.TRAVEL, DroneMode.SEARCH)
                target_pos = command["sector_start_pos"]
                self.set_drone_target_pos(target_pos.lat, target_pos.lon)
                with self._lock:
                    self.pathfinder = PathfinderState(target_pos, None)
        
        self.set_drone_last_command(drone_command)
            
    def connect(self):
        with self._lock:
            self.drone_states[self.drone_id]._mode = DroneMode.IDLE
            self.drone_states[self.drone_id]._battery_percentage = random.uniform(10, 100)
            self.log(f"Reports READY")

    def set_drone_mode(self, new_mode: DroneMode, next_mode: DroneMode = None):
        with self._lock:
            self.drone_states[self.drone_id]._mode = new_mode
            if next_mode is not None:
                self.next_mode = next_mode

    def set_drone_battery_percentage(self, new_battery_percentage: float):
        with self._lock:
            self.drone_states[self.drone_id]._battery_percentage = new_battery_percentage

    def set_drone_estimated_rtt(self, new_estimated_rtt: float):
        with self._lock:
            self.drone_states[self.drone_id]._estimated_rtt = new_estimated_rtt

    def set_drone_position(self, new_lat: float, new_lon: float):
        with self._lock:
            self.drone_states[self.drone_id]._position = LatLon(new_lat, new_lon)

    def set_drone_battery_percentage(self, new_battery_percentage: float):
        with self._lock:
            self.drone_states[self.drone_id]._battery_percentage = new_battery_percentage

    def set_drone_target_pos(self, new_lat: float, new_lon: float):
        with self._lock:
            self.target_pos = LatLon(new_lat, new_lon)

    def set_drone_last_command(self, command: DroneCommand):
        with self._lock:
            self.drone_states[self.drone_id]._last_command = command
    
class DroneSystem(metaclass=_SingletonMeta):
    """
    This runs in a separate thread to simulate both the mission_control_node and the drones.
    """
    def __init__(self, drone_states: Dict[DroneId, DroneState], start_pos: LatLon):
        self.drones: Dict[DroneId, Drone] = {}
        self.commands: Queue[Tuple[DroneId, DroneCommand]] = Queue()  # Simulated command queue from mission control node to drones
        for drone_id in drone_states.keys():
            self.drones[drone_id] = Drone(drone_id, drone_states, start_pos)

        self.modification_lock = Lock()
        self.exit = Event()
    
    def start(self):
        """ A thread that simulates drone movements """
        self._t = Thread(target=self._start, daemon=True)
        self._t.start()

    def connect_drones(self):
        for drone in self.drones.values():
            drone.connect()

    def _start(self):
        while not self.exit.is_set():
            # Step each drone
            for drone in self.drones.values():
                drone.cycle()

            # Check for commands
            while not self.commands.empty():
                drone_id, drone_command = self.commands.get()
                self.drones[drone_id].handle_command(drone_command)

            sleep(DRONE_CYCLE_INTERVAL)

    def add_command(self, drone_id: DroneId, drone_command: DroneCommand):
        self.commands.put_nowait((drone_id, drone_command))

    

