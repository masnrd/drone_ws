# full_launcher

A ROS2 launch script that automatically launches the given number of drones.
- This spawns a given number of drones (`SIM_DRONE_COUNT`) around the latitude and longitude provided (`PX4_HOME_LAT`, `PX4_HOME_LON`).
    - We can't spawn the drones in the exact same spot because of collision detection.
    - These settings are defined in `default_settings.sh` in the root directory of the ROS2 workspace.