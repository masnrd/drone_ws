from pathlib import Path
from sys import stderr

ROS_ROOT = Path("/opt/ros")
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