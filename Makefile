DRONE_DEP = $(wildcard src/drone/drone/*.py)
MC_DEP = $(wildcard src/mission_control/mission_control/*.py)
SENSOR_DEP = $(wildcard src/sensor_node/sensor_node/*.py)
LAUNCH_DEP = $(wildcard src/full_launcher/full_launcher/*.py)

build_launch: $(LAUNCH_DEP)
	colcon build --packages-select full_launcher

# Builds drone node for ROS 2
build_drone: $(DRONE_DEP)
	colcon build --packages-select drone

build_sensor: $(SENSOR_DEP)
	colcon build --packages-select sensor_node

# Runs Python tests for the drone
test_drone: $(DRONE_DEP)
	colcon test-result --delete-yes
	colcon test --packages-select drone
	colcon test-result --all --verbose