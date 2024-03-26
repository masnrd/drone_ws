from typing import Dict
from pathlib import Path
from lib import *

DEFAULT_COORDS = {
    "latitude": 1.3410943577604117,
    "longitude": 103.96540423849946,
    "altitude": -999.0
}

APP = "setup_real_env"

def generate_default_env(proj_root: Path, ros_install: Path, coords: Dict[str, float], ros_domain_id: int):
    state = f"[{APP}: Generate env.sh]"
    contents = ""

    # Auto source all necessary things
    contents += f"source {str(ros_install.absolute())}\n"
    contents += "source ./install/setup.bash\n"
    contents += "source ./drone_config.sh\n\n"

    # Get drones
    drones_str = input("Enter the IDs of the drones, separated by commas: ")
    drone_ids = []
    for drone_str in drones_str.split(","):
        try:
            drone_id = int(drone_str.strip("[] \t"))
        except ValueError:
            error(f"Invalid drone ID: {drone_str}")
        drone_ids.append(drone_id)

    drone_ids_str = "[" + ", ".join([str(id) for id in drone_ids]) + "]"

    # Add environment variables
    lat = coords.get("latitude", 0.0)
    lon = coords.get("longitude", 0.0)
    alt = coords.get("altitude", 0.0)

    contents += f"export MC_MODE=1\n" # for real
    contents += f"export DRONE_IDS=\"{drone_ids_str}\"\n"
    contents += f"export PX4_HOME_LAT={lat}\nexport PX4_HOME_LON={lon}\nexport PX4_HOME_ALT={alt}\n"
    contents += f"export ROS_DOMAIN_ID={ros_domain_id}\n"

    try:
        with proj_root.joinpath(ENV_NAME).open("w") as fp:
            fp.write(contents)
    except Exception:
        error(state, f"Error writing to {proj_root.joinpath(ENV_NAME).absolute()}.")

def main(proj_root: Path):
    state = f"[{APP}: Init]"
    report(state, "Attempting to find ROS installation...")
    ros_install_path = get_ros_install(ROS_ROOT)
    report(state, f"Located ROS install file: {ros_install_path}")
    ros_domain_id = -1
    ros_domain_id_str = input("Enter ROS Domain ID: ")
    while True:
        valid = True
        try:
            ros_domain_id = int(ros_domain_id_str)
            if ros_domain_id not in range(0, 233):
                valid = False
        except ValueError:
            valid = False

        if valid:
            break
        ros_domain_id_str = input(f"Invalid ROS Domain ID \"{ros_domain_id_str}\", please enter an integer between 0 and 232: ")
    report(state, f"ROS_DOMAIN_ID set to {ros_domain_id}")

    # Validation of ROS2 install
    state = f"[{APP}: Input validation]"
    if not ros_install_path.is_file():
        error(state, f"Error: Could not locate ROS setup.bash file ({ros_install_path}). Ensure ROS2 is installed.")

    # 1. Generate default env
    state = f"[{APP}: Generate env.sh]"
    report(state, "Generating file...")
    generate_default_env(proj_root, ros_install_path, DEFAULT_COORDS, ros_domain_id)
    report(state, f"env.sh file generated: {proj_root.joinpath(ENV_NAME).absolute()}")

    # 3. Check builds
    state = f"[{APP}: Build Checks]"
    ## If project is not built
    project_is_built = False
    if proj_root.joinpath("build").exists():
        expected = []
        for p in proj_root.joinpath("src").iterdir():
            if not p.is_dir():
                continue
            if p.name == "px4_msgs":  # skip this, they need to rebuild anyway
                continue
            expected.append(str(proj_root.joinpath("build").joinpath(p.name).absolute()))
        result = []
        for p in proj_root.joinpath("build").iterdir():
            if not p.is_dir():
                continue
            if p.name == "px4_msgs":
                continue
            result.append(str(p.absolute()))

        if sorted(result) == sorted(expected):
            project_is_built = True

    ## Report warnings
    if not project_is_built:
        warn(state, "`drone_ws` workspace has not been built. Run the following command in the project root (drone_ws) before attempting to run any simulations:")
        printcommand(f"source {ros_install_path}")
        printcommand("colcon build")
    else:
        warn(state, "`px4_msgs` was updated. Run the following command in the project root (drone_ws) before attempting to run any simulations:")
        printcommand(f"source {ros_install_path}")
        printcommand("colcon build --packages-select px4_msgs")

    state = f"[{APP}: Completed]"
    report(state, "Setup completed.")

if __name__ == "__main__":
    proj_root = get_project_root()
    main(proj_root)