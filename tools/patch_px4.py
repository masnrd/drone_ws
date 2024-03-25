import subprocess
from lib import *

APP = "patch_px4"

def patch_px4_autopilot(proj_root: Path, px4_autopilot_path: Path):
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

def main(proj_root: Path):
    px4_autopilot_path = get_px4_autopilot_path(APP)

    # Patch PX4-Autopilot
    state = f"[{APP}: PX4-Autopilot Patch]"
    report(state, "Patching PX4-Autopilot..")
    patch_px4_autopilot(proj_root, px4_autopilot_path)
    report(state, "PX4-Autopilot patched successfully.")

if __name__ == "__main__":
    proj_root = get_project_root()
    main(proj_root)