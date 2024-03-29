# drone_ws
ROS workspace for:
- Drone (`./src/drone`)
    - Drone Sensor (`./src/sensor_node`)
    - Launcher for multiple drones in a simulated environment (`./src/full_launcher`)
- Mission Control (`./src/mission_control`)

In addition, this includes:
- Mission Control User Interface (`./frontend`)
- Messages for the interface between the Mission Control and Drones (`./src/mc_interface_msgs`)
- Messages for the interface between the Drone and Sensor Node (`./src/sensor_interface_msgs`)

## Installation
Follow the instructions provided [here](https://docs.px4.io/main/en/ros/ros2_comm.html) to install ROS2, PX4-Autopilot, and the MicroXRCEAgent.
- Here are the repositories for reference:
    - [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot). When running the `ubuntu.sh` script as expected, do not include any flags.
    - [MicroXRCEAgent](https://github.com/eProsima/Micro-XRCE-DDS-Agent): Installation steps are [here](https://docs.px4.io/main/en/middleware/uxrce_dds.html).
- You will also need the `PX4-gazebo-models` repository: [PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models)

Clone this repository along with all submodules:
```
git clone --recursive https://github.com/masnrd/drone_ws.git
```

Install all Python3 dependencies:
```bash
# In the project root (i.e. drone_ws)
python3 -m pip install -r requirements.txt
```
- If you wish to run the UI, run the following command:
    ```bash
    # In the ./frontend directory
    npm install
    ```
- To use the custom world:
    1. Install the default worlds and models:
        ```bash
        python3 simulation-gazebo --overwrite
        ```
    2. Follow the instructions [here](./worlds/README.md) to install the custom worlds used for simulation.

Run the setup script, which updates PX4-Autopilot with the necessary files:
```bash
# In the project root (i.e. drone_ws)
python3 ./tools/patch_px4.py
```
- You **may** need to re-run the `ubuntu.sh` setup file in the `PX4-Autopilot/Tools` directory.

Now, build the ROS2 project.
```bash
# In the project root (i.e. drone_ws)
source /opt/ros/iron/setup.bash
colcon build
```

Inspect the `drone_config.sh` script, which contains the parameters for the drone node.
- This will be automatically sourced by `env.sh`, so there is no need to manually source this.

---

## Usage (Physical)
Setup the real `env.sh` to connect to the drones. This will:
- Request for the drone IDs to connect to.
- Request for the desired `ROS_DOMAIN_ID` to use.

```bash
# In the project root
python3 ./tools/setup_real_env.py
```

Initialise the Mission Control end of the Telemetry Communication.
```bash
sudo pppd -detach nocrtscts lock noauth passive persist holdoff 3 maxfail 0 local [local IP]:[remote IP] [device] 57600
```

Run the Mission Control:
- **Mission Control Backend**: This is the interface between the GUI and the ROS2 node communicating with the drones.
    ```bash
    # In the project root
    source ./env.sh     # Generated by ./tools/setup_real_env.py
    ros2 run mission_control mission_control_node
    ```
- **Mission Control Frontend**: This runs the GUI React frontend.
    ```bash
    # In the ./frontend directory
    npm start
    ```

The drones should be using the corresponding `stable/drone-v*` branch, and should automatically connect to Mission Control.

---

## Usage (Simulation, Software-in-the-Loop (SITL))
### Setup
Ensure that your PX4 SITL build exists. This command generates a build for the simulated flight controller.
```bash
# In your PX4-Autopilot directory
make px4_sitl
```

Then, generate the `env.sh` for the simulated environment.
```bash
# In the project root
python3 ./tools/setup_sim_env.py
```

### Generating a New Sensor Map
This would generate a new map of simulated mobile devices in `./src/sensor_node/simmaps`.
```bash
# In the project root directory
PX4_HOME_LAT = (target latitude)
PX4_HOME_LON = (target longitude)
python3 ./tools/sim_generator.py [device count] [max latitude offset] [min latitude offset]
```
- Note that `env.sh` already defines `PX4_HOME_LAT` and `PX4_HOME_LON`, you can simply call `source env.sh`.

To preview a map `./src/sensor_node/simmaps/x.simmap`:
```bash
# In the project root directory
python3 ./tools/sim_generator.py x.simmap
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

**Drone Simulation**: In a **third** terminal, run the launch file, which launches both the necessary simulated flight controllerss, the ROS2 drone nodes, and the simulated sensor nodes.
```bash
# In the project root directory
source ./env.sh  # Generated by ./setup_env/setup_env.py
ros2 launch full_launcher drone_launch.py
```

**Mission Control Backend**: In a **fourth** terminal, run the Mission Control backend. This launches the ROS2 node that connects to the drones, and the Flask web server that runs the API backend. 
```bash
# In the project root directory
source ./env.sh  # Generated by ./setup_env/setup_env.py
ros2 run mission_control mission_control_node
```

**Mission Control Frontend**: Finally, in a fifth terminal, run the Mission Control React frontend.
```bash
# In the ./frontend directory
npm start
```
- You can connect to this at `127.0.0.1:3000`. This opens a map showing the locations of each drone.
    - Right now, the simulated field in Gazebo isn't exactly aligned with its position due to its rotation -- hence the position of the simulated drone won't exactly appear to match with the drone on the frontend.
    - Another TODO is to make the React frontend use the environment variable `PX4_HOME_LAT` and `PX4_HOME_LON` to set the start position of the map.
