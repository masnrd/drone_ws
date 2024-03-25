# drone_ws
ROS workspace for the Jetson Nano itself.

## Installation

Clone this repository along with all submodules:
```
git clone --recursive https://github.com/masnrd/drone_ws.git -b release/drone-v0.7-safe-fsm --single-branch
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

Now, build the ROS2 project.
```bash
# In the project root (i.e. drone_ws)
source /opt/ros/foxy/setup.bash
colcon build
```

---

## Usage
Setup `env.sh` to connect to the mission control. This will:
- Request for the drone ID for this drone.
- Request for the desired `ROS_DOMAIN_ID` to use.

```bash
# In the project root
python3 ./tools/setup_env.py
```

Initialise the drone end of the Telemetry Communication. This should preferably be implemented using a `systemctl` target.
```bash
sudo pppd -detach nocrtscts lock noauth passive persist holdoff 3 maxfail 0 local [local IP]:[remote IP] [device] 57600
```

Finally, start the drone. This launcher script will automatically start both the drone main node and the sensor node:
```bash
# In the project root
source env.sh
ros2 launch full_launcher drone_launch.py
```

The mission control should be using the corresponding `release/mc-v*` branch, and the drone should automatically connect to it.
