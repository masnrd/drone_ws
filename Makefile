DEP = $(wildcard src/drone/drone/*.py)

# Builds drone node for ROS 2
build: $(DEP)
	colcon build --packages-select drone

# Runs Python tests
test: $(DEP)
	colcon test-result --delete-yes
	colcon test --packages-select drone
	colcon test-result --all --verbose