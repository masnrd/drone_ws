# drone_ws
ROS workspace for the Jetson Nano itself.

## Installation
### Configuration for the PX4 Flight Controller
Using QGroundControl or the NuttX Shell, modify the following parameters:
- `MAV_0_CONFIG`: `DISABLED`.
- `UXRCE_DDS_CFG`: `TELEM2`.
    - This ensures that the PX4 flight controller sends and receives messages using UXRCE instead of MavLink, and over `TELEM2`.
- `SER_TEL2_BAND`: `115200 8N1`.
    - Note that this option, if missing, should reappear after setting the above two parameters and rebooting.
 
The connection between the Jetson Nano and the flight controller should be:
- `TELEM2` on the Flight Controller
- `UART1` on the Jetson Nano (hence `/dev/ttyTHS1`).

### Configuration for the Jetson Nano
Follow the instructions provided [here](https://docs.px4.io/main/en/ros/ros2_comm.html) to install ROS2 and the MicroXRCEAgent.
- [MicroXRCEAgent](https://github.com/eProsima/Micro-XRCE-DDS-Agent): Installation steps are [here](https://docs.px4.io/main/en/middleware/uxrce_dds.html).

Clone this repository along with all submodules:
```
git clone --recursive https://github.com/masnrd/drone_ws.git -b release/drone-v0.8-speedy-gonzales --single-branch
```

Install all Python3 dependencies:
```bash
# In the project root (i.e. drone_ws)
python3 -m pip install -r requirements.txt
```

Run the setup script, which updates the PX4 messages with the necessary files:
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

Ensure MicroXRCEAgent is active.
```bash
sudo MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 115200
```
- Note that the values given to `--dev` and `-b` are dependent on how you connected the flight controller to the Jetson, and the baud rate you set in QGC for the controller!

Finally, start the drone. This launcher script will automatically start both the drone main node and the sensor node:
```bash
# In the project root
source env.sh
ros2 launch full_launcher drone_launch.py
```

The mission control should be using the corresponding `release/mc-v0.8-speedy-gonzales` branch, and the drone should automatically connect to it.

---

## Validating the Connection between Jetson Nano and Flight Controller
You may wish to validate that the Jetson Nano is correctly receiving messages from the Flight Controller.

1. Start `MicroXRCEAgent` on the Jetson Nano.
    ```bash
    sudo MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 115200
    ```
2. In a separate console on the Jetson Nano:
    ```bash
    # In project root
    source env.sh   # To have the PX4 messages
    ros2 topic list # This should show PX4-specific topics
    ```
3. In the output, ensure you can see PX4-specific ROS2 topics like `/fmu/out/vehicle_status`.
4. If you wish, you can see the output:
   ```bash
   ros2 topic echo /fmu/out/vehicle_status
   ```
    
