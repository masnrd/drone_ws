# masnrd_ws

Workspace for the Jetson Nano itself. This does not contain the code for Mission Control, only for the drone.

## Installation
Clone this repository along with all submodules:
```
git clone --recursive https://github.com/masnrd/drone_ws.git
```

Install all Python3 dependencies:
```bash
# In the project root (i.e. drone_ws)
python3 -m pip install -r requirements.txt
```

Run the setup script, which updates `./src/px4_msgs` with the necessary files and generates an `env.sh` file for further usage:
```bash
# In the project root (i.e. drone_ws)
python3 ./setup_env/setup_env.py
```
- **Note**: This automatically sources the necessary ROS2 `setup.bash` and the local installation `setup.bash` files, so sourcing this is all you need when running ROS2 files later on :)

Now, build the ROS2 project.
```bash
# In the project root (i.e. drone_ws)
source /opt/ros/foxy/setup.bash
colcon build
```

## Usage
```bash
# In the drone_ws directory
ros2 run drone drone_node -p droneId:=[drone id]
```