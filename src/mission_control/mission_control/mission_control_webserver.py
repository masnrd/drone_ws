"""
mission_control_webserver:
A Flask webserver that:
- Provides a REST API to interact with drones.
    - When a route is called, the route function generates the necessary command with the `DroneCommand` API,
    and places it in the command queue in `self.commands`. This command queue is accessed every second (by default).
    - Drone information is obtained by accessing the getters or `toJSON()` methods of the drone states
- Provides a web client to interact with the API.
    - The web client is provided in the `static` directory provided in the package.
"""
# UNCOMMENT FOR PYTHON3.8
# from __future__ import annotations
from flask import Flask, request, send_from_directory
from ament_index_python import get_package_share_directory
from typing import Dict, Tuple
from queue import Queue
from pathlib import Path
from .drone_utils import DroneState, DroneId
from .drone_utils import DroneCommand, DroneCommand_SEARCH_SECTOR, DroneCommand_RTB, DroneCommand_MOVE_TO
from .maplib import LatLon

class MCWebServer:
    def __init__(self, drone_states: Dict[int, DroneState], commands: Queue[Tuple[DroneId, DroneCommand]]):
        self.static_dir = Path(get_package_share_directory("mission_control")).joinpath("frontend")
        self.app = Flask(
            "Mission Control", 
            static_url_path='',
            static_folder=self.static_dir,
            template_folder=self.static_dir,
        )
        self.drone_states = drone_states
        self.commands: Queue[Tuple[DroneId, DroneCommand]] = commands

        # Set up Endpoints
        self.app.add_url_rule("/", view_func=self.route_index)
        self.app.add_url_rule("/api/info", view_func=self.route_info)
        self.app.add_url_rule("/api/action/search", view_func=self.route_action_search)

    def drone_exists(self, drone_id: DroneId):
        return drone_id in self.drone_states.keys()

    def route_index(self):
        return send_from_directory(self.static_dir, "index.html")

    def route_info(self) -> Dict[int, str]:
        ret: Dict[int, str] = {}
        for drone_id, drone_state in self.drone_states.items():
            ret[drone_id] = drone_state.toJSON()
        
        return ret
    
    def route_action_search(self) -> Dict[int, str]:
        drone_id = request.args.get("drone_id", type=int, default=None)
        lat = request.args.get("lat", type=float, default=None)
        lon = request.args.get("lon", type=float, default=None)

        if drone_id is None:
            return {"error": "need drone_id"}, 400
        
        if not self.drone_exists(drone_id):
            return {"error": f"no such drone with drone_id {drone_id}"}, 400

        if lat is None or lon is None:
            return {"error": "need latitude/longitude parameters as float"}, 400
        
        # Place command in command queue
        command_tup = (drone_id, DroneCommand_SEARCH_SECTOR(LatLon(lat, lon), None))
        self.commands.put_nowait(command_tup)
        
        return {}, 200
    
    def route_action_moveto(self) -> Dict[int, str]:
        #TODO
        return {"error": "not implemented"}, 500
    
    def run(self):
        self.app.run(debug=True, use_reloader=False)