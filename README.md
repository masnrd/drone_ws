# masnrd_ws

A ROS workspace for the onboard computer. 

## Installation
Clone this repository along with all submodules:
```
git clone --recursive https://github.com/masnrd/drone_ws.git
```

Install all dependencies with:
```bash
python3 -m pip install -r requirements.txt  # Installs all Python3 dependencies for the ROS2 mission control and drone nodes.
npm install                                 # Installs all Node dependencies for the React frontend, not needed if you're not running that.
```

If the ROS2 distribution hasn't been initialised before, do so:
```bash
source /opt/ros/iron/setup.bash
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

**Server for Drone Simulation**: In a **second** terminal, start the Gazebo server.
```bash
# In the PX4-gazebo-models directory
python3 simulation-gazebo --world SUTD_field
```

**Drone Simulation**: In a **third** terminal, run the launch file, which launches both the necessary simulated flight controllers and the ROS2 drone nodes.
```bash
# In the drone_ws directory
source ./install/setup.bash
source default_settings.sh   # To ensure environment variables are consistent with mission control
ros2 launch full_launcher drone_launch.py
```
- Warning: Since it takes a while for the flight controller to actually reach the READY state, the drone node may give up connecting before actually connecting. Just restart for now.


**Mission Control Backend**: In a **fourth** terminal, run the Mission Control backend. This launches the ROS2 node that connects to the drones, and the Flask web server that runs the API backend. 
```bash
# In the drone_ws directory
source ./install/setup.bash
source default_settings.sh   # Environment vars like start location, drone count
ros2 run mission_control mission_control_node
```

**Mission Control Frontend**: Finally, in a fifth terminal, run the Mission Control React frontend.
```bash
# In the drone_ws directory
npm start
```
- You can connect to this at `127.0.0.1:3000`. This opens a map showing the locations of each drone.
    - Right now, the simulated field in Gazebo isn't exactly aligned with its position due to its rotation -- hence the position of the simulated drone won't exactly appear to match with the drone on the frontend.
    - Another TODO is to make the React frontend use the environment variable `PX4_HOME_LAT` and `PX4_HOME_LON` to set the start position of the map.
- Clustering can be done at `127.0.0.1:3000/cluster`.
