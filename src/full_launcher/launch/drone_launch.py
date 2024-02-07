import launch
import launch_ros.actions
import launch_ros.substitutions
import random
from typing import Tuple
from pathlib import Path

START_LATLON = (1.340643554050367, 103.9626564184675)  #TODO: change to env var/input?
PX4_AUTOPILOT_PATH = Path("/home/rye/Documents/Capstone/PX4-Autopilot") #TODO: change to env var

def deterministic_xy(drone_id: int, min: float, max: float) -> Tuple[float, float]:
    """ Returns a deterministic X and Y position based on the drone_id """
    random.seed(drone_id); random.seed(random.getrandbits(16))
    return (random.uniform(min, max), random.uniform(min, max))

def generate_drone(fc_sitl_build_path: Path, drone_id: int, rel_x, rel_y) -> Tuple[launch.LaunchDescriptionEntity, launch_ros.actions.Node]:
    fc = launch.actions.ExecuteProcess(
        cmd=[
            launch.substitutions.FindExecutable(name=str(fc_sitl_build_path.absolute())),
            f"-i {drone_id}"
        ],
        additional_env={
            "PX4_SYS_AUTOSTART": "4001",
            "PX4_SIM_MODEL": "gz_x500",
            "PX4_HOME_LAT": str(START_LATLON[0]),
            "PX4_HOME_LON": str(START_LATLON[1]),
            "PX4_GZ_MODEL_POSE": f"{rel_x},{rel_y}"
        },
        shell=True
    )
    obc = launch_ros.actions.Node(
            namespace=f"mc_{drone_id}",
            package="drone",
            executable="drone_node",
            ros_arguments=[
                "-p", f"droneId:={str(drone_id)}"
            ]
        )
    return (fc, obc)

def generate_launch_description():
    # Ensure build exists
    fc_sitl_build_path = PX4_AUTOPILOT_PATH.joinpath("build/px4_sitl_default/bin/px4")
    print(f"PX4_SITL_BUILD_PATH: {fc_sitl_build_path}")
    if not fc_sitl_build_path.exists():
        print("PX4 SITL build not accessible, please run `make px4_sitl` in your PX4-Autopilot directory and update the path in line 7 accordingly.")
        exit(1)
    launch_entities = []

    drone_count = 10
    
    for drone_id in range(1, drone_count+1):
        rel_x, rel_y = deterministic_xy(drone_id, -2.0, 2.0)
        drone_tup = generate_drone(fc_sitl_build_path, drone_id, rel_x, rel_y)
        launch_entities.append(drone_tup[0])
        launch_entities.append(drone_tup[1])

    return launch.LaunchDescription(launch_entities)