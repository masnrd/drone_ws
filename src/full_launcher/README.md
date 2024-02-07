# full_launcher

A ROS2 launch script that automatically launches the given number of drones.
- Currently, all settings are within the `launch/drone_launch.py` file. This spawns a given number of drones around the provided latitude and longitude.
    - We can't spawn the drones in the exact same spot because of collision detection.

## Setup
First, replace line 7 with the filepath of your `PX4-Autopilot` directory.

Within that directory, run:
```bash
make px4_sitl
```

Then, with `drone_ws` as your current directory, run either one of the following two commands:
```bash
colcon build       # If you haven't built the project before
make build_launch  # If you just want to build the launcher
```

## Usage
In a separate terminal in the `PX4-gazebo-models` directory, start the Gazebo server.
```bash
python simulation-gazebo --world SUTD_field
```

Back in the `drone_ws` directory, source the installation.
```bash
source ./install/setup.bash
```

Run the mission control node separately in another terminal.
```bash
ros2 run mission_control mission_control_node
```
- NOTE: Right now, this only handles 2 drones. It's still in TODO.

Run the launch file, which would launch both the flight controllers and the drone nodes.
```bash
ros2 launch full_launcher drone_launch.py
```
- Warning: Since it takes a while for the flight controller to actually reach the READY state, the drone node may give up connecting before actually connecting. Just restart for now.

## TODO
- Read command line args for:
    - Number of drones to spawn
- Read environment variables for:
    - Beginning latitude and longitude
    - PX4-Autopilot path
- Spawn the mission control as well, need to modify the mission control node to handle more than 2 drones.
- Increase the timeout for flight controller connection.
