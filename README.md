# masnrd_ws

A ROS workspace for the onboard computer. See [IMPLEMENTATION](./docs/IMPLEMENTATION.md) for the planned implementation.

## Installation
If the ROS2 distribution hasn't been initialised before, do so:
```bash
source /opt/ros/foxy/setup.bash # or your preferred ROS2 distribution
```

In the project root:
```bash
colcon build
```

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

Having initialised the environment, run the central interface node and the pathfinder node on a third terminal:
```bash
ros2 run drone drone_node
``````
