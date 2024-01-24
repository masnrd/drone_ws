import threading
import logging
from queue import Queue, Empty
from typing import Dict, Tuple

from maplib import LatLon
from drone_utils import DroneId, DroneState, DroneCommand, DroneMode, DroneCommandId
from mission_control_webserver import MCWebServer
from fake_drone_system import DroneSystem

logging.basicConfig(encoding='utf-8', level=logging.DEBUG)

COMMAND_CHECK_INTERVAL = 1
HOME_POSITION = LatLon(1.3399775009363866, 103.96258672159254)


class MCNode:
    def __init__(self, drone_states: Dict[DroneId, DroneState], commands: Queue[Tuple[DroneId, DroneCommand]]):
        self.logger = logging.getLogger("mission_control")
        self.drone_sys = DroneSystem(drone_states, HOME_POSITION)
        self.drone_states = drone_states

        # Initialise command queue
        self.command_loop = threading.Timer(COMMAND_CHECK_INTERVAL, self.check_command_loop)
        self.commands: Queue[Tuple[DroneId, DroneCommand]] = commands
        self.logger.info("Mission Control initialised.")

    def start_node(self):
        self.command_loop.start()

    def log(self, msg: str):
        self.logger.info(f"MISSION CONTROL: {msg}")

    def raise_error(self, msg: str):
        self.logger.error(f"MISSION CONTROL ERROR: {msg}")
        raise Exception(msg)
    
    def check_command_loop(self):
        """ Loop to check for commands from the webserver """
        try:
            drone_id, command = self.commands.get(block=False)
            print(f"MISSION CONTROL: User entered command {DroneCommandId(command.command_id).name}")
            if drone_id not in self.drone_states.keys():
                print(f"Warning: No such drone ID {drone_id}")
                return
            self.mc_send_command(drone_id, command)
        except Empty:
            pass
        self.command_loop = threading.Timer(COMMAND_CHECK_INTERVAL, self.check_command_loop)
        self.command_loop.start()

    def mc_send_command(self, drone_id: DroneId, drone_cmd: DroneCommand):
        """ MC sends a Command to a given drone """
        self.drone_sys.add_command(drone_id, drone_cmd)


def main():
    drone_states = {
        DroneId(69): DroneState(69),
        DroneId(1): DroneState(1),
        DroneId(2): DroneState(2),
        DroneId(3): DroneState(3),
        DroneId(4): DroneState(4),
    }
    commands: Queue[Tuple[DroneId, DroneCommand]] = Queue()

    # Start web server
    webserver = MCWebServer(drone_states, commands)
    webserver_thread = threading.Thread(target=webserver.run, daemon=True)
    webserver_thread.start()

    # Start rclpy node
    mcnode = MCNode(drone_states, commands)
    mcnode.start_node()

    # Start drones
    drone_sys = DroneSystem(drone_states, HOME_POSITION)
    drone_sys.start()
    drone_sys.connect_drones()

    try:
        while True:
            user_in = input() # nth done here, just to avoid busy waiting
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
    finally:
        drone_sys.exit.set()


if __name__ == '__main__':
    main()