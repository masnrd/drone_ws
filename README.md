# masnrd_ws

A ROS workspace for the onboard computer. 
- See [DRONE_IMPLEMENTATION](./docs/DRONE_IMPLEMENTATION.md) for the planned implementation for the drone.

## Installation
Clone this repository along with all submodules:
```
git clone --recursive https://github.com/masnrd/drone_ws.git
```

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
- Build launcher: `make build_launch`

For simulation, you will need the following repositories and to have performed the necessary installation steps:
- [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)
- [MicroXRCEAgent](https://github.com/eProsima/Micro-XRCE-DDS-Agent): Installation steps are [here](https://docs.px4.io/main/en/middleware/uxrce_dds.html)
- [PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models)

In the `PX4-gazebo-models` repository, install the default worlds and models:
```bash
python3 simulation-gazebo
```

Next, back in this repository, follow the instructions in [here](./worlds/README.md) to install the custom worlds used for simulation.

## Usage
### Setup
Modify the default settings in `./default_settings.sh` -- for instance, you need to set the path to your PX4-Autopilot directory.

Ensure that your PX4 SITL build exists. This command generates a build for the simulated flight controller.
```bash
# In your PX4-Autopilot directory
make px4_sitl
```

### Running Simulation
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
ros2 run mission_co)ntrol mission_control_node
```

In a **fourth** terminal, run the launch file, which launches both the necessary simulated flight controllers and the ROS2 drone nodes.
```bash
# In the drone_ws directory
source ./install/setup.bash
source default_settings.sh   # To ensure environment variables are consistent with mission control
ros2 launch full_launcher drone_launch.py
```
- Warning: Since it takes a while for the flight controller to actually reach the READY state, the drone node may give up connecting before actually connecting. Just restart for now.

You can connect to the web application at `127.0.0.1:5000`.
- Two endpoints are provided on the Flask backend.
    - `/api/info`: Returns a dump of the state of every drone connected (just 1 drone for now).
    - `/api/action/search?drone_id=[drone_id]&lat=[latitude]&lon=[longitude]`: Provides a `SEARCH_SECTOR` command to drone `drone_id`, starting at `latitude, longitude`.
        - It is recommended to choose a latitude and longitude close to the starting point of the drone.
        - Note that the command is dropped if the drone is not connected to mission control, this is still in progress.
- For now, the frontend is simply a table displaying the state of all drones (defined by the `drone_states` dictionary), and polls the `/api/info` endpoint every second.
