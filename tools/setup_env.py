import subprocess
from sys import stderr
from typing import Dict
from pathlib import Path

PROJECT_ROOT = None
if Path.cwd().stem != "drone_ws":
    if Path.cwd().parent.stem == "drone_ws":
        PROJECT_ROOT = Path.cwd().parent
    else:
        print("Could not locate project root.")
        exit(1)
else:
    PROJECT_ROOT = Path.cwd()

ROS_ROOT = Path("/opt/ros")
ENV_NAME = "env.sh"
DEFAULT_COORDS = {
    "latitude": 1.3410943577604117,
    "longitude": 103.96540423849946,
    "altitude": -999.0
}
DEFAULT_DRONE_COUNT = 2

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
    state = "[setup_env: locate ROS]"
    if not ros_root.is_dir():
        error(state, f"Could not locate ROS installation in {ros_root}.")
    installs = []
    for p in ros_root.iterdir():
        if p.is_dir():
            installs.append(p)
    if len(installs) == 0:
        error(state, f"Could not locate ROS installation in {ros_root}.")
    
    install_dir_str = installs[0]
    if len(installs) > 1:
        tgt_i = -1
        report(state, f"Located multiple ROS install directories:")
        for i, install in enumerate(installs):
            print(f"{i}: {install}")
        while tgt_i < 0 or tgt_i >= len(installs):
            tgt_i = input(f"Enter the index of the desired install (0-{len(installs)-1}): ")
        install_dir_str = installs[tgt_i]

    # Get setup.bash
    install_file = Path(install_dir_str).joinpath("setup.bash")

    return install_file.resolve()

def patch_px4_autopilot(proj_root: Path, px4_autopilot_path: Path):
    state = "[setup_env: PX4 Patch: Downgrade PX4-Autopilot]"
    # Revert PX4-Autopilot
    make_result = subprocess.run(["make", "clean"], cwd=px4_autopilot_path)
    if make_result.returncode != 0:
        error(state, f"Error in running `make clean` in {px4_autopilot_path}. Ensure you have actually pulled the PX4-Autopilot repository. Error: {make_result.stderr}")
    subprocess.run(["make", "distclean"], cwd=px4_autopilot_path)

    git_result = subprocess.run(["git", "fetch", "origin", "release/1.14"], cwd=px4_autopilot_path)
    if git_result.returncode != 0:
        error(state, f"Error in running `git fetch origin release/1.14` in {px4_autopilot_path}. Ensure you have actually pulled the PX4-Autopilot repository. Error: {git_result.stderr}")
    checkout_result = subprocess.run(["git", "checkout", "release/1.14"], cwd=px4_autopilot_path)
    if checkout_result.returncode != 0:
        error(state, f"Error in running `git checkout release/1.14` in {px4_autopilot_path}. : {checkout_result.stderr}")
        print(f"[{px4_autopilot_path}] Error in running `git checkout release/1.14`: {checkout_result.stderr}")
        exit(1)
    subprocess.run(["make", "submodulesclean"], cwd=px4_autopilot_path)
    report(state, "PX4-Autopilot successfully set to release/1.14.")

    # Patch .simulator file
    state = "[setup_env: PX4 Patch: Patch simulator file]"
    target_dir = px4_autopilot_path.joinpath("ROMFS").joinpath("px4fmu_common").joinpath("init.d-posix")
    old_simulator_file = target_dir.joinpath("px4-rc.simulator")
    new_simulator_file = PROJECT_ROOT.joinpath("setup_env").joinpath("px4-rc.simulator")
    if not target_dir.is_dir():
        error(state, f"Could not find directory in path {old_simulator_file}. Please ensure you've successfully downloaded the PX4-Autopilot directory, otherwise inform Ryan of this error.")
    if not new_simulator_file.is_file():
        error(state, f"Could not find px4-rc.simulator file in path {new_simulator_file}. Please inform Ryan of this error.")

    ## Attempt to remove px4-rc.simulator
    try:
        old_simulator_file.unlink(missing_ok=True)
    except PermissionError:
        error(state, f"No permissions to remove {old_simulator_file}")

    ## Attempt to add the new version of the simulator file
    try:
        old_simulator_file.touch()
        with old_simulator_file.open("w") as dst:
            with new_simulator_file.open("r") as src:
                dst.write(src.read())
    except PermissionError:
        error(state, f"No permissions to write from {new_simulator_file} to {old_simulator_file}")
    except Exception as e:
        error(state, f"Error when writing from {new_simulator_file} to {old_simulator_file}: {e}")
    report(state, "PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator patched.")

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

    ## Update messages
    try:
        src_msg_dir = px4_autopilot_path.joinpath("msg")
        dst_msg_dir = px4_msgs_path.joinpath("msg")
        for p in dst_msg_dir.glob("*.msg"):
            p.unlink()
        for src_p in src_msg_dir.glob("*.msg"):
            dst_p = dst_msg_dir.joinpath(src_p.name)
            dst_p.touch(exist_ok=True)
            with dst_p.open("w") as dst:
                with src_p.open("r") as src:
                    dst.write(src.read())
    except PermissionError as e:
        error(state, f"Error with permissions when updating messages: {e}")
    except Exception as e:
        error(state, f"Unexpected Error when updating messages: {e}")
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
     
def generate_default_env(proj_root: Path, ros_install: Path, px4_autopilot_dir: Path, coords: Dict[str, float], drone_count: int):
    state = "[setup_env: Generate env.sh]"
    contents = ""

    # Auto source all necessary things
    contents += f"source {str(ros_install.absolute())}\n"
    contents += "source ./install/setup.bash\n\n"

    # Add environment variables
    lat = coords.get("latitude", 0.0)
    lon = coords.get("longitude", 0.0)
    alt = coords.get("altitude", 0.0)
    
    contents += f"export PX4_AUTOPILOT_PATH=\"{str(px4_autopilot_dir.absolute())}\"\n"
    contents += f"export PX4_HOME_LAT={lat}\nexport PX4_HOME_LON={lon}\nexport PX4_HOME_ALT={alt}\n"
    contents += f"export SIM_DRONE_COUNT={drone_count}\n\n"

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
    report(state, "Attempting to find ROS installation...")
    ros_install_path = get_ros_install(ROS_ROOT)
    report(state, f"Located ROS install file: {ros_install_path}")

    px4_autopilot_path_str = input("Enter the path of PX4-Autopilot: ")
    px4_autopilot_path_str = px4_autopilot_path_str.replace("~", str(Path.home().absolute()))
    px4_autopilot_path = Path(px4_autopilot_path_str).resolve()
    report(state, f"PX4-Autopilot path: {px4_autopilot_path}")

    # Validation of ROS2 install and PX4-Autopilot
    state = "[setup_env: Input validation]"
    if not px4_autopilot_path.is_dir():
        error(state, f"Path given was not a directory ({px4_autopilot_path})")

    if not ros_install_path.is_file():
        error(state, f"Error: Could not locate ROS setup.bash file ({ros_install_path}). Ensure ROS2 is installed.")

    # 1. Patch PX4-Autopilot
    state = "[setup_env: PX4-Autopilot Patch]"
    ## Revert to release branch
    report(state, "Patching PX4-Autopilot..")
    patch_px4_autopilot(PROJECT_ROOT, px4_autopilot_path)
    report(state, "PX4-Autopilot patched successfully.")
    
    # 2. Generate default env
    state = "[setup_env: Generate env.sh]"
    report(state, "Generating file...")
    generate_default_env(PROJECT_ROOT, ros_install_path, px4_autopilot_path, DEFAULT_COORDS, DEFAULT_DRONE_COUNT)
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
    ## If px4_sitl is not built
    px4_sitl_is_built = False
    if px4_autopilot_path.joinpath("build").exists():
        build_bin = px4_autopilot_path.joinpath("build").joinpath("px4_sitl_default").joinpath("bin").joinpath("px4")
        if build_bin.is_file():
            px4_sitl_is_built = True

    ## Report warnings
    if not project_is_built:
        warn(state, "`drone_ws` workspace has not been built. Run the following command in the project root (drone_ws) before attempting to run any simulations:")
        printcommand(f"source {ros_install_path}")
        printcommand("colcon build")
    else:
        warn(state, "`px4_msgs` was updated. Run the following command in the project root (drone_ws) before attempting to run any simulations:")
        printcommand(f"source {ros_install_path}")
        printcommand("colcon build --packages-select px4_msgs")
    if not px4_sitl_is_built:
        warn(state, f"Could not locate an SITL build for PX4-Autopilot. Run the following command in the PX4-Autopilot directory before attempting to run any simulations.")
        printcommand("make px4_sitl")

    state = "[setup_env: Completed]"
    report(state, "Setup completed.")


if __name__ == "__main__":
    main()