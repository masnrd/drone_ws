from pathlib import Path
from sys import stderr
from os import environ

ROS_ROOT = Path("/opt/ros")

def get_project_root() -> Path:
    proj_root = Path(__file__).parent.parent
    if Path.cwd().stem != "drone_ws":
        if Path.cwd().parent.stem == "drone_ws":
            proj_root = Path.cwd().parent
        else:
            print("Could not locate project root.")
            exit(1)
    else:
        proj_root = Path.cwd()
    report("[lib: Init]", f"Project Root set to: {proj_root}")
    return proj_root

def get_px4_autopilot_path(app: str) -> Path:
    state = f"[{app}: Init]"
    px4_autopilot_path_str = environ.get("PX4_AUTOPILOT_PATH", None)
    if px4_autopilot_path_str is not None:
        user_input = input(f"Detected existing PX4_Autopilot path \"{px4_autopilot_path_str}\", use that? (Y/N): ").lower()
        while user_input != "y" and user_input != "n":
            user_input = input(f"Invalid input, enter 'Y' or 'N': ").lower()
        
        if user_input == "n":
            px4_autopilot_path_str = None
    
    if px4_autopilot_path_str is None:
        px4_autopilot_path_str = input("Enter the path of PX4-Autopilot: ")
        
    px4_autopilot_path_str = px4_autopilot_path_str.replace("~", str(Path.home().absolute()))

    px4_autopilot_path = Path(px4_autopilot_path_str).resolve()
    report(state, f"PX4-Autopilot path: {px4_autopilot_path}")

    # Validation of PX4-Autopilot
    state = f"[{app}: Input validation]"
    if not px4_autopilot_path.is_dir():
        error(state, f"Path given was not a directory ({px4_autopilot_path})")
    return px4_autopilot_path

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

def rm_dir(dir: Path):
    if not dir.is_dir():
        return
    for p in dir.iterdir():
        if p.is_dir():
            rm_dir(p)
        else:
            p.unlink(missing_ok=True)
    dir.rmdir()


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