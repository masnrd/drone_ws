from pathlib import Path
from lib import *

APP = "setup_real_env"
     
def generate_default_env(proj_root: Path, ros_install: Path):
    state = f"[{APP}: Generate env.sh]"
    contents = ""

    # Auto source all necessary things
    contents += f"source {str(ros_install.absolute())}\n"
    contents += "source ./install/setup.bash\n\n"

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

    contents += f"export MC_MODE=1\n" # for real
    contents += f"export DRONE_IDS=\"{drone_ids_str}\"\n"

    try:
        with proj_root.joinpath(ENV_NAME).open("w") as fp:
            fp.write(contents)
    except Exception:
        error(state, f"Error writing to {proj_root.joinpath(ENV_NAME).absolute()}.")

def main():
    state = f"[{APP}: Init]"
    report(state, "Attempting to find ROS installation...")
    ros_install_path = get_ros_install(ROS_ROOT)
    report(state, f"Located ROS install file: {ros_install_path}")

    # Validation of ROS2 install and PX4-Autopilot
    state = f"[{APP}: Input validation]"
    if not ros_install_path.is_file():
        error(state, f"Error: Could not locate ROS setup.bash file ({ros_install_path}). Ensure ROS2 is installed.")
    
    # 1. Generate default env
    state = f"[{APP}: Generate env.sh]"
    report(state, "Generating file...")
    generate_default_env(PROJECT_ROOT, ros_install_path)
    report(state, f"env.sh file generated: {PROJECT_ROOT.joinpath(ENV_NAME).absolute()}")

    # 3. Check builds
    state = f"[{APP}: Build Checks]"
    ## If project is not built
    project_is_built = False
    if PROJECT_ROOT.joinpath("build").exists():
        expected = []
        for p in PROJECT_ROOT.joinpath("src").iterdir():
            if not p.is_dir():
                continue
            if p.name == "px4_msgs":  # skip this, they need to rebuild anyway
                continue
            expected.append(str(PROJECT_ROOT.joinpath("build").joinpath(p.name).absolute()))
        result = []
        for p in PROJECT_ROOT.joinpath("build").iterdir():
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
    main()