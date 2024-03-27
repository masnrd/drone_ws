import launch
import launch_ros.actions
import launch_ros.substitutions
import random
from os import environ
from typing import Tuple
from pathlib import Path

def deterministic_xy(drone_id: int, min: float, max: float) -> Tuple[float, float]:
    """ Returns a deterministic X and Y position based on the drone_id """
    random.seed(drone_id); random.seed(random.getrandbits(16))
    return (random.uniform(min, max), random.uniform(min, max))

def generate_drone(drone_id: int) -> Tuple[launch_ros.actions.Node, launch_ros.actions.Node]:
    obc = launch_ros.actions.Node(
        namespace=f"mc_{drone_id}",
        package="drone",
        executable="drone_node",
        arguments=[
            "--ros-args", "-p", f"droneId:={str(drone_id)}"
        ]
    )
    
    sensor_node = launch_ros.actions.Node(
        namespace=f"mc_{drone_id}",
        package="sensor_node",
        executable="sensor_node",
        arguments=[
            "--ros-args", "-p", f"droneId:={str(drone_id)}"
        ]
    )

    return (obc, sensor_node)

def generate_launch_description():
    # Load env vars
    drone_id_str = environ.get("DRONE_ID")
    drone_id = -1
    try:
        drone_id = int(drone_id_str)
    except ValueError:
        drone_id = -1

    if drone_id not in range(1, 233):
        print("Invalid drone ID, please re-run `./tools/setup_env.py` and `source env.sh`.")
        exit(1)
    
    drone_tup = generate_drone(drone_id)
    launch_entities = [drone_tup[0], drone_tup[1]]
    print(f"Initialising Drone {drone_id}.")

    return launch.LaunchDescription(launch_entities)