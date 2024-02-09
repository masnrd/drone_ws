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

def generate_drone(fc_sitl_build_path: Path, drone_id: int, home_lat: float, home_lon: float, rel_x: float, rel_y: float) -> Tuple[launch.LaunchDescriptionEntity, launch_ros.actions.Node]:
    fc = launch.actions.ExecuteProcess(
        cmd=[
            launch.substitutions.FindExecutable(name=str(fc_sitl_build_path.absolute())),
            f"-i {drone_id}"
        ],
        additional_env={
            "PX4_SYS_AUTOSTART": "4001",
            "PX4_SIM_MODEL": "gz_x500",
            "PX4_HOME_LAT": str(home_lat),
            "PX4_HOME_LON": str(home_lon),
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
    # Load env vars
    px4_autopilot_path_str = environ.get("PX4_AUTOPILOT_PATH", None)
    if px4_autopilot_path_str is None:
        print("\"PX4_AUTOPILOT_PATH\" environment variable does not exist, please define one linking to the PX4-Autopilot directory.")
        exit(1)
    start_lat, start_lon = float(environ.get("PX4_HOME_LAT", 0.0)), float(environ.get("PX4_HOME_LON", 0.0))
    drone_count = int(environ.get("SIM_DRONE_COUNT", "2"))

    # Ensure build exists
    px4_autopilot_path = Path(px4_autopilot_path_str)
    fc_sitl_build_path = px4_autopilot_path.joinpath("build/px4_sitl_default/bin/px4")
    print(f"PX4_SITL_BUILD_PATH: {fc_sitl_build_path}")
    if not fc_sitl_build_path.exists():
        print("PX4 SITL build not accessible, please run `make px4_sitl` in your PX4-Autopilot directory and update the path in line 7 accordingly.")
        exit(1)
    launch_entities = []

    # Evaluate start location
    print(f"Using Start Location: ({start_lat}, {start_lon})")
    print(f"Drone Count: {drone_count}")
    
    for drone_id in range(1, drone_count+1):
        rel_x, rel_y = deterministic_xy(drone_id, -2.0, 2.0)
        drone_tup = generate_drone(fc_sitl_build_path, drone_id, start_lat, start_lon, rel_x, rel_y)
        launch_entities.append(drone_tup[0])
        launch_entities.append(drone_tup[1])

    return launch.LaunchDescription(launch_entities)