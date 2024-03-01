import subprocess
from sys import stderr
from typing import Dict
from pathlib import Path

ROS_ROOT = Path("/opt/ros/iron")
PROJECT_ROOT = Path(__file__).parent.parent
ENV_NAME = "env.sh"

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def report(state: str, msg: str):
    print(f"{bcolors.OKBLUE}{state} {msg}{bcolors.ENDC}")

def printcommand(msg: str):
    print(f"\t{bcolors.WARNING}{bcolors.BOLD}{msg}{bcolors.ENDC}")

def warn(state: str, msg: str):
    print(f"{bcolors.WARNING}{state} WARNING: {msg}{bcolors.ENDC}", file=stderr)

def error(state: str, msg: str):
    print(f"{bcolors.FAIL}{state} ERROR: {msg}{bcolors.ENDC}", file=stderr)
    exit(1)

def get_ros_install(ros_root: Path) -> Path:
    state = "[setup_env: locate ROS foxy]"
    if not ros_root.is_dir():
        error(state, f"Could not locate ROS foxy installation in {ros_root}.")

    # Get setup.bash
    install_file = Path(ros_root).joinpath("setup.bash")

    return install_file.resolve()

def patch_px4_autopilot(proj_root: Path):
    # Patch px4-msgs
    state = "[setup_env: PX4 Patch: Downgrade px4_msgs]"
    px4_msgs_path = proj_root.joinpath("src").joinpath("px4_msgs")
    ## Ensure it exists in the first place
    if not px4_msgs_path.joinpath("README.md").is_file():
        error(state, f"Error: Could not identify the px4_msgs repo in {px4_msgs_path}. Please ensure you included the `--recursive` flag when cloning the `drone_ws` repository.")
    ## Rollback to release/1.14
    git_result = subprocess.run(["git", "fetch", "origin", "release/1.14"], cwd=px4_msgs_path)
    if git_result.returncode != 0:
        error(state, f"Error in running `git fetch origin release/1.14` in {px4_msgs_path}: {git_result.stderr}")
    checkout_result = subprocess.run(["git", "checkout", "release/1.14"], cwd=px4_msgs_path)
    if checkout_result.returncode != 0:
        error(state, f"Error in running `git checkout release/1.14` in {px4_msgs_path}: {checkout_result.stderr}")

    report(state, "px4_msgs updated.")

    ## Delete builds for px4_msgs
    state = "[setup_env: PX4 Patch: Delete px4_msg build]"
    try:
        build_dir = proj_root.joinpath("build").joinpath("px4_msgs")
        installs_dir = proj_root.joinpath("install").joinpath("px4_msgs")
        if build_dir.is_dir():
            rm_dir(build_dir)
        if installs_dir.is_dir():
            rm_dir(installs_dir)
    except PermissionError as e:
        error(state, f"Error with permissions when deleting build: {e}")
    except Exception as e:
        error(state, f"Unexpected Error when deleting build: {e}")
    report(state, "px4_msgs original build removed.")
     
def generate_default_env(proj_root: Path, ros_install: Path, drone_id: int):
    state = "[setup_env: Generate env.sh]"
    contents = ""

    # Auto source all necessary things
    contents += f"source {str(ros_install.absolute())}\n"
    contents += "source ./install/setup.bash\n\n"

    contents += f"export DRONE_ID={drone_id}\n\n"

    contents += "echo \"Environment initialised. Run the following command to start the drone:\"\n"
    contents += "echo \"ros2 run drone drone_node --ros-args -p droneId:=$DRONE_ID\"\n"

    try:
        with proj_root.joinpath(ENV_NAME).open("w") as fp:
            fp.write(contents)
    except Exception:
        error(state, f"Error writing to {proj_root.joinpath(ENV_NAME).absolute()}.")

def rm_dir(dir: Path):
    if not dir.is_dir():
        return
    for p in dir.iterdir():
        if p.is_dir():
            rm_dir(p)
        else:
            p.unlink(missing_ok=True)
    dir.rmdir()

def main():
    state = "[setup_env: Init]"
    drone_id_str = input("Enter the intended ID of this drone: ")
    try:
        drone_id = int(drone_id_str)
    except ValueError:
        error(state, f"Invalid drone ID given: {drone_id}")

    report(state, "Attempting to find ROS installation...")
    ros_install_path = get_ros_install(ROS_ROOT)
    report(state, f"Located ROS install file: {ros_install_path}")

    # Validation of ROS2 install and PX4-Autopilot
    state = "[setup_env: Input validation]"
    if not ros_install_path.is_file():
        error(state, f"Error: Could not locate ROS setup.bash file ({ros_install_path}). Ensure ROS2 is installed.")

    # 1. Patch PX4-Autopilot
    state = "[setup_env: px4_msgs Patch]"
    ## Revert to release branch
    report(state, "Patching px4_msgs...")
    patch_px4_autopilot(PROJECT_ROOT)
    report(state, "px4_msgs patched successfully.")
    
    # 2. Generate default env
    state = "[setup_env: Generate env.sh]"
    report(state, "Generating file...")
    generate_default_env(PROJECT_ROOT, ros_install_path, drone_id)
    report(state, f"env.sh file generated: {PROJECT_ROOT.joinpath(ENV_NAME).absolute()}")

    # 3. Check builds
    state = "[setup_env: Build Checks]"
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

    state = "[setup_env: Completed]"
    report(state, "Setup completed.")


if __name__ == "__main__":
    main()