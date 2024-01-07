# CONSTANTS
START_LAT = 1.40724
START_LON = 104.02896

DEP = $(wildcard src/drone/drone/*.py)

# Builds drone node for ROS 2
build: $(DEP)
	colcon build --packages-select drone

# Setup simulator coordinates
sim_prep:
	export PX4_HOME_LAT=$(START_LAT)
	export PX4_HOME_LON=$(START_LON)

# Runs Python tests
test: $(DEP)
	colcon test-result --delete-yes
	colcon test --packages-select drone
	colcon test-result --all --verbose