DRONE_DEP = $(wildcard src/drone/drone/*.py)
MC_DEP = $(wildcard src/mission_control/mission_control/*.py)

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