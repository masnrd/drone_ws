# dummy_mission_control

A dummy mission control that runs the Mission Control web server without any dependencies on ROS2.
- Previous dependencies on ROS are now replaced with a stub.
- As part of the simulation, drones are simulated in a separate thread, receiving commands as necessary.

# Installation
```bash
pip install -r requirements.txt
```

# Usage
```bash
python3 mission_control_node.py
```

This will initialise the web app at `127.0.0.1:5000`.
- You can send commands to drones via the endpoints:
    - `/api/info`: Obtains drone information, this is used by the index page to query drone status.
    - `/api/action/moveto?drone_id={DRONE ID}&lat={LATITUDE}&lon={LONGITUDE}`: Move to a position, then idle.
    - `/api/action/search?drone_id={DRONE ID}&lat={LATITUDE}&lon={LONGITUDE}`: Move to a position, then begin search at that coordinate.
- Note that if you set a faraway latitude and longitude, the drone WILL start going in that direction, so do choose a coordinate close by.

To set the starting coordinates of all drones, modify `HOME_POSITION` in line 14 of `mission_control_node.py`.

# Organisation
The main webapp content is in `mission_control_webserver.py`.
- This is deliberately done in a class in order to allow us to run the Flask webserver in a separate thread, spawned by `mission_control_node`.
- New routes can be defined as shown in `MCWebServer.__init__()`.
- The static files can be found in `frontend/`.
    - The web client polls the `/api/info` endpoint every second to update.
    - I'm not too good at JavaScript, but the contents of `frontend/js/main.js` show how to interact with the JSON data from `/api/info`.

The main starting point of the code is in `mission_control_node`.
- This spawns the webserver as a thread, runs the simulated mission control node, and spawns the simulated drone system in a separate thread.

Other components of the code:
- `drone_utils.py`: Utilities used by both the mission control node and the drones.
- `maplib.py`: Utilities for computing things related to latitude and longitude.
- `fake_drone_system.py`: A simulated system of drones to simulate the drone swarm the mission control node interacts with.
- `pathfinder.py`: The pathfinding code, used by the simulated drones (WIP).