# Builds drone node for ROS 2
drone:
	colcon build --packages-select drone

# Runs Python tests
test:
	colcon test-result --delete-yes
	colcon test --packages-select drone
	colcon test-result --all --verbose