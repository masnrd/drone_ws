DRONE_DEP = $(wildcard src/drone/drone/*.py)
MC_DEP = $(wildcard src/mission_control/mission_control/*.py)
LAUNCH_DEP = $(wildcard src/full_launcher/full_launcher/*.py)

build_launch: $(LAUNCH_DEP)
	colcon build --packages-select full_launcher

# Builds drone node for ROS 2
build_drone: $(DRONE_DEP)
	colcon build --packages-select drone

# Builds mission control node for ROS 2
build_mc: $(MC_DEP)
	colcon build --packages-select mission_control

# Runs Python tests for the drone
test_drone: $(DRONE_DEP)
	colcon test-result --delete-yes
	colcon test --packages-select drone
	colcon test-result --all --verbose

# Runs Python tests for the drone
test_mc: $(MC_DEP)
	colcon test-result --delete-yes
	colcon test --packages-select mission_control
	colcon test-result --all --verbose

# Inform user if they run this invalid command
px4_sitl:
	@echo "\033[91mPlease run this command in the PX4-Autopilot directory instead.\033[0m"