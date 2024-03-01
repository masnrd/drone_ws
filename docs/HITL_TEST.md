# HITL Testing

## Setup
PX4 Flight Controller should be connected serially to the companion.
- On the flight controller, the companion computer should be `/dev/ttyACM0`, if connected via microUSB. Otherwise, refer to [Pixhawk6C Serial Port Mapping](https://docs.px4.io/main/en/flight_controller/pixhawk6c.html#serial-port-mapping).
- On the companion computer, the flight controller should be `/dev/ttyACM0` or `/dev/ttyACM1` -- it should be possible to check this with `dmesg`.

We first need to start the micro XRCE-DDS client on that port. First, enter the NuttX shell via MavLink:
```bash
# In the PX4-Autopilot directory
bash$ ./Tools/mavlink_shell.py /dev/ttyACM0
Connecting to MAVLINK...

# List devices connected to the flight controller
nsh> ls /dev
/dev:
  ...
  ttyACM0      # this should be our companion computer!
  ...

# Stop microXRCE DDS Client
nsh> uxrce_dds_client stop

# Ensure microXRCE DDS Client is stopped
nsh> uxrce_dds_client status
INFO  [uxrce_dds_client] not running

# Start microXRCE DDS Client on the serial port
nsh> uxrce_dds_client start -t serial -d /dev/ttyACM0
nsh> uxrce_dds_client status
INFO  [uxrce_dds_client] Running, disconnected
INFO  [uxrce_dds_client] Using transport: serial

# Disconnect MavLink on serial port
nsh> mavlink stop -d /dev/ttyACM0  # This should disconnect you from the flight controller's shell, don't panic
device reports readiness to read but returned no data (device disconnected or multiple access on port?)
```

Next, start the micro XRCE-DDS agent on the companion computer to begin receiving and sending data to the flight controller:
```bash
MicroXRCEAgent serial --dev /dev/ttyACM0 -v 4
```

You should be able to see `create_client`, `establish_session` on the agent's output.

Now, start the drone node on the companion computer:
```bash
ros2 run drone drone_node
```

## Troubleshooting
If the MicroXRCEAgent's output is as expected, then the problem is with the flight controller not arming. This could require manually disabling the safety on the GPS module.

The problem can be further diagnosed by inspecting the ROS2 topic list:
```bash
ros2 topic list
```
- If the GPS module isn't fixed, then `/fmu/out/vehicle_global_position` will not appear, which our drone relies on to work.

If MicroXRCEAgent's output is *not* as expected (i.e. it gets stuck on 2-3 lines), then the connection with the flight controller is the issue.