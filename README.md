# masnrd_ws

A ROS workspace for the onboard computer. 
- See [DRONE_IMPLEMENTATION](./docs/DRONE_IMPLEMENTATION.md) for the planned implementation for the drone.

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
First, initialise the micro XRCE-DDS agent and generate the necessary simulation files (within the `PX4-Autopilot` directory):
```bash
MicroXRCEAgent udp4 -p 8888
make px4_sitl
```

#### Mission Control Initialisation
For each drone you wish to initialise, add a new entry in the `drone_states` dictionary in line 96 of `mission_control_node`, and run `build_mc` within the `drone_ws` workspace.

Having done the necessary `source` commands, start the Mission Control server.
```bash
ros2 run mission_control mission_control_node
```
- This will also run the Flask web server on `127.0.0.1:5000`.

#### Drone Initialisation
For each drone with ID `droneId` to be initialised, start a new terminal.
- To define the starting latitude and longitude:
    ```bash
    export PX4_HOME_LAT=[latitude]
    export PX4_HOME_LON=[longitude]
    ```

In the `PX4_Autopilot` workspace, having run `make px4_sitl`, initialise the PX4 SITL stack with the Gazebo simulator:
```bash
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i [droneId]
```
- This will run the Gazebo simulator with a drone if it's not open yet. Otherwise, it will initialise a new drone on the simulator.

Then, initialise the drone ROS2 node on another terminal.
```bash
ros2 run drone drone_node --ros-args -p droneId:=[droneId]
```
- The drone node will first connect to the flight controller, then report `READY` to mission control, before consistently updating its status.
- On the mission control frontend, you should be able to see live updates from each drone.


### Web Application
#### Backend
Currently, two endpoints are provided on the Flask backend.
- `/api/info`: Returns a dump of the state of every drone connected (just 1 drone for now).
- `/api/action/search?drone_id=[drone_id]&lat=[latitude]&lon=[longitude]`: Provides a `SEARCH_SECTOR` command to drone `drone_id`, starting at `latitude, longitude`.
    - The only supported drone ID is `69` for now.
    - It is recommended to choose a latitude and longitude close to the starting point of the drone.
    - Note that the command is dropped if the drone is not connected to mission control, this is still in progress.

#### Frontend
The frontend can be accessed at `127.0.0.1:5000`. For now, this is simply a table displaying the state of all drones (just 1 drone for now), and polls the `/api/info` endpoint every second.