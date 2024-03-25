from typing import Dict
from pathlib import Path
from lib import *

APP = "setup_env"

def generate_default_env(proj_root: Path, ros_install: Path, ros_domain_id: int, drone_id: int):
    state = f"[{APP}: Generate env.sh]"
    contents = ""

    # Auto source all necessary things
    contents += f"source {str(ros_install.absolute())}\n"
    contents += "source ./install/setup.bash\n\n"

    contents += f"export DRONE_ID={drone_id}\n"
    contents += f"export ROS_DOMAIN_ID={ros_domain_id}\n\n"

    contents += "echo \"Environment initialised. Run the following command to start the drone:\"\n"
    contents += "echo \"ros2 launch full_launcher drone_launch.py\"\n"
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

    drone_id = -1
    drone_id_str = input("Enter drone ID: ")
    while True:
        valid = True
        try:
            drone_id = int(drone_id_str)
            if drone_id not in range(1, 233):
                valid = False
        except ValueError:
            valid = False

        if valid:
            break
        drone_id_str = input(f"Invalid Drone ID \"{drone_id_str}\", please enter an integer between 1 and 232.")
    report(state, f"ROS_DOMAIN_ID set to {ros_domain_id}")
    report(state, f"Drone ID set to {drone_id}")


    # Validation of ROS2 install
    state = f"[{APP}: Input validation]"
    if not ros_install_path.is_file():
        error(state, f"Error: Could not locate ROS setup.bash file ({ros_install_path}). Ensure ROS2 is installed.")

    # 1. Generate default env
    state = f"[{APP}: Generate env.sh]"
    report(state, "Generating file...")
    generate_default_env(proj_root, ros_install_path, ros_domain_id, drone_id)
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

    state = f"[{APP}: Completed]"
    report(state, "Setup completed.")

if __name__ == "__main__":
    proj_root = get_project_root()
    main(proj_root)