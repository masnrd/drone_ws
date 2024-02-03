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
import logging
import json
from flask import Flask, request, send_from_directory, Response
from typing import Dict, Tuple
from queue import Queue
from pathlib import Path

from mission_utils import Mission
from drone_utils import DroneState, DroneId
from drone_utils import DroneCommand, DroneCommand_SEARCH_SECTOR, DroneCommand_RTB, DroneCommand_MOVE_TO
from run_clustering import run_clustering
from maplib import LatLon

class MCWebServer:
    def __init__(self, mission:Mission, drone_states: Dict[int, DroneState], commands: Queue[Tuple[DroneId, DroneCommand]]):
        self.static_dir = Path("frontend")
        # Flask.logger_name = "listlogger"
        self.app = Flask(
            "Mission Control", 
            static_url_path='',
            static_folder=self.static_dir,
            template_folder=self.static_dir,
        )

        self.mission = mission
        self.drone_states = drone_states
        self.commands: Queue[Tuple[DroneId, DroneCommand]] = commands

        # Set up Endpoints
        self.app.add_url_rule("/", view_func=self.route_index)
        self.app.add_url_rule("/api/info", view_func=self.route_info)
        self.app.add_url_rule("/api/action/moveto", view_func=self.route_action_moveto)
        self.app.add_url_rule("/api/action/search", view_func=self.route_action_search)
        self.app.add_url_rule("/api/setup/run_clustering", methods=["POST"], view_func=self.route_run_clustering)
        self.app.after_request(self.add_headers)

    def route_index(self):
        return send_from_directory(self.static_dir, "index.html")

    def route_info(self) -> Dict[str, str]:
        ret: Dict[str, Dict]
        drones: Dict[int, str] = {}

        ret: Dict[str, str] = {}
        for drone_id, drone_state in self.drone_states.items():
            drones[drone_id] = drone_state.toJSON()
        ret["mission"] = self.mission.__dict__
        ret["drones"] = drones
        return ret
    
    def route_action_moveto(self) -> Tuple[Dict, int]:
        drone_id = request.args.get("drone_id", type=int, default=None)
        lat = request.args.get("lat", type=float, default=None)
        lon = request.args.get("lon", type=float, default=None)

        if drone_id is None:
            return {"error": "need drone_id"}, 400

        if lat is None or lon is None:
            return {"error": "need latitude/longitude parameters as float"}, 400
        
        # Place command in command queue
        command_tup = (drone_id, DroneCommand_MOVE_TO(LatLon(lat, lon)))
        self.commands.put_nowait(command_tup)
        
        return {}, 200
    
    def route_action_search(self) -> Tuple[Dict, int]:
        drone_id = request.args.get("drone_id", type=int, default=None)
        lat = request.args.get("lat", type=float, default=None)
        lon = request.args.get("lon", type=float, default=None)

        if drone_id is None:
            return {"error": "need drone_id"}, 400

        if lat is None or lon is None:
            return {"error": "need latitude/longitude parameters as float"}, 400
        
        # Place command in command queue
        command_tup = (drone_id, DroneCommand_SEARCH_SECTOR(LatLon(lat, lon), None))
        self.commands.put_nowait(command_tup)
        
        return {}, 200

    def route_run_clustering(self):
        data = request.form.to_dict()
        hotspots_location = data.get("hotspots_position", None)
        cluster_centres = run_clustering(json.loads(hotspots_location))
        return cluster_centres
    
    def add_headers(self, response: Response):
        response.headers.add("Content-Type", "application/json")
        response.headers.add("Access-Control-Allow-Methods", "PUT, GET ,POST, DELETE, OPTIONS")
        response.headers.add("Access-Control-Allow-Origin", "*")
        response.status = 200
        return response
    
    def run(self):
        self.app.run(debug=True, use_reloader=False)