import subprocess
from lib import *

PROJECT_ROOT = Path(__file__).parent.parent
APP = "patch_px4"

def patch_px4_autopilot(proj_root: Path, px4_autopilot_path: Path):
    state = f"[{APP}: PX4 Patch: Downgrade PX4-Autopilot]"
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
    state = f"[{APP}: PX4 Patch: Patch simulator file]"
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
    state = f"[{APP}: PX4 Patch: Downgrade px4_msgs]"
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
    state = f"[{APP}: PX4 Patch: Delete px4_msg build]"
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

def main():
    state = f"[{APP}: Init]"

    px4_autopilot_path_str = input("Enter the path of PX4-Autopilot: ")
    px4_autopilot_path_str = px4_autopilot_path_str.replace("~", str(Path.home().absolute()))
    px4_autopilot_path = Path(px4_autopilot_path_str).resolve()
    report(state, f"PX4-Autopilot path: {px4_autopilot_path}")

    # Validation of PX4-Autopilot
    state = f"[{APP}: Input validation]"
    if not px4_autopilot_path.is_dir():
        error(state, f"Path given was not a directory ({px4_autopilot_path})")

    # Patch PX4-Autopilot
    state = f"[{APP}: PX4-Autopilot Patch]"
    report(state, "Patching PX4-Autopilot..")
    patch_px4_autopilot(PROJECT_ROOT, px4_autopilot_path)
    report(state, "PX4-Autopilot patched successfully.")