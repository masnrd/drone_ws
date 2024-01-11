# masnrd_ws

A ROS workspace for the onboard computer. See [IMPLEMENTATION](./docs/IMPLEMENTATION.md) for the planned implementation.

## Installation
Install all Python dependencies with:
```
python3 -m pip install -r requirements.txt
```

If the ROS2 distribution hasn't been initialised before, do so:
```bash
source /opt/ros/foxy/setup.bash # or your preferred ROS2 distribution
```

In the project root:
```bash
colcon build
```

## Development
Assuming you have built the project before (under Installation), you can simply run the following make targets:
- Build Drone node: `make build_drone`
- Build Mission Control node: `make build_mc`
- Test Drone node: `make test_drone`
- Test Mission Control node: `make test_mc`

## Usage
### Initialising the Environment
If the ROS2 distribution hasn't been initialised before, do so:
```bash
source /opt/ros/foxy/setup.bash # or your preferred ROS2 distribution
```

Initialise the environment with:
```bash
source install/local_setup.bash
```
- This provides access to the environment hooks for this workspace.
- This is an overlay on top of the underlying ROS2 environment (having sourced `setup.bash` for the ROS2 distribution earlier)

### SITL Testing
In a separate terminal, initialise the PX4 SITL stack with the simulator:
```bash
make px4_sitl jmavsim
```

In another terminal, initialise the micro XRCE-DDS agent:
```bash
MicroXRCEAgent udp4 -p 8888
```

First, initialise mission control on one terminal.
```bash
ros2 run mission_control mission_control_node
```

Then, initialise the drone (with hardcoded ID `69`) on another terminal. The drone node will first connect to the flight controller, then report `READY` to mission control, before consistently updating its status.
```bash
ros2 run drone drone_node
``````
