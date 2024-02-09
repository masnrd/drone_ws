# full_launcher

A ROS2 launch script that automatically launches the given number of drones.
- This spawns a given number of drones (`SIM_DRONE_COUNT`) around the latitude and longitude provided (`PX4_HOME_LAT`, `PX4_HOME_LON`).
    - We can't spawn the drones in the exact same spot because of collision detection.
    - These settings are defined in `default_settings.sh` in the root directory of the ROS2 workspace.

## Setup
Modify the default settings in `./default_settings.sh` -- for instance, you need to set the path to your PX4-Autopilot directory.

First, ensure that your PX4 SITL build exists. This command generates a build for the simulated flight controller.
```bash
# In your PX4-Autopilot directory
make px4_sitl
```

Then, with `drone_ws` as your current directory, run **either one** of the following two commands:
```bash
# In the drone_ws directory
colcon build       # If you haven't built the project before
make build_launch  # If you just want to build the launcher
```

## Usage
To ensure communication works, start the Micro XRCE-DDS Agent.
```bash
# In the Micro-XRCE-DDS-Agent directory
MicroXRCEAgent udp4 -p 8888
```

In a **second** terminal, start the Gazebo server.
```bash
# In the PX4-gazebo-models directory
python simulation-gazebo --world SUTD_field
```

In **third** terminal, run the Mission Control node. 
```bash
# In the drone_ws directory
source ./install/setup.bash
source default_settings.sh   # Environment vars like start location, drone count
ros2 run mission_control mission_control_node
```

In a **fourth** terminal, run the launch file, which launches both the necessary simulated flight controllers and the ROS2 drone nodes.
```bash
# In the drone_ws directory
source ./install/setup.bash
source default_settings.sh   # To ensure environment variables are consistent with mission control
ros2 launch full_launcher drone_launch.py
```
- Warning: Since it takes a while for the flight controller to actually reach the READY state, the drone node may give up connecting before actually connecting. Just restart for now.

## TODO
- Increase the timeout for flight controller connection.
