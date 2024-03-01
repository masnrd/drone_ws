from typing import Dict
from pathlib import Path
from lib import *

DEFAULT_COORDS = {
    "latitude": 1.3410943577604117,
    "longitude": 103.96540423849946,
    "altitude": -999.0
}
DEFAULT_DRONE_COUNT = 2
APP = "setup_sim_env"
     
def generate_default_env(proj_root: Path, ros_install: Path, px4_autopilot_dir: Path, coords: Dict[str, float], drone_count: int):
    state = f"[{APP}: Generate env.sh]"
    contents = ""

    # Auto source all necessary things
    contents += f"source {str(ros_install.absolute())}\n"
    contents += "source ./install/setup.bash\n\n"

    # Add environment variables
    lat = coords.get("latitude", 0.0)
    lon = coords.get("longitude", 0.0)
    alt = coords.get("altitude", 0.0)
    
    contents += f"export MC_MODE=0\n" # for simulation
    contents += f"export PX4_AUTOPILOT_PATH=\"{str(px4_autopilot_dir.absolute())}\"\n"
    contents += f"export PX4_HOME_LAT={lat}\nexport PX4_HOME_LON={lon}\nexport PX4_HOME_ALT={alt}\n"
    contents += f"export SIM_DRONE_COUNT={drone_count}\n\n"

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

    px4_autopilot_path_str = input("Enter the path of PX4-Autopilot: ")
    px4_autopilot_path_str = px4_autopilot_path_str.replace("~", str(Path.home().absolute()))
    px4_autopilot_path = Path(px4_autopilot_path_str).resolve()
    report(state, f"PX4-Autopilot path: {px4_autopilot_path}")

    # Validation of ROS2 install and PX4-Autopilot
    state = f"[{APP}: Input validation]"
    if not px4_autopilot_path.is_dir():
        error(state, f"Path given was not a directory ({px4_autopilot_path})")
    state = f"[{APP}: Input validation]"
    if not ros_install_path.is_file():
        error(state, f"Error: Could not locate ROS setup.bash file ({ros_install_path}). Ensure ROS2 is installed.")
    
    # 1. Generate default env
    state = f"[{APP}: Generate env.sh]"
    report(state, "Generating file...")
    generate_default_env(PROJECT_ROOT, ros_install_path, px4_autopilot_path, DEFAULT_COORDS, DEFAULT_DRONE_COUNT)
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

    state = f"[{APP}: Completed]"
    report(state, "Setup completed.")


if __name__ == "__main__":
    main()