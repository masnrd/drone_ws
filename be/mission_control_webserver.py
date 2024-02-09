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
from flask import Flask, jsonify, request, send_from_directory, Response
from typing import Dict, Tuple
from queue import Queue
from pathlib import Path
from assigner.simplequeueassigner import SimpleQueueAssigner

from mission_utils import Mission
from drone_utils import DroneState, DroneId
from drone_utils import DroneCommand, DroneCommand_SEARCH_SECTOR, DroneCommand_MOVE_TO
from run_clustering import run_clustering
from maplib import LatLon
from flask_cors import CORS

logging.getLogger("flask_cors").level = logging.ERROR
logging.getLogger("werkzeug").level = logging.ERROR

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
        CORS(self.app)

        self.mission = mission
        self.drone_states = drone_states
        self.commands: Queue[Tuple[DroneId, DroneCommand]] = commands
        self.assigner = SimpleQueueAssigner()

        # Set up Endpoints
        self.app.add_url_rule("/", view_func=self.route_index)
        self.app.add_url_rule("/api/info", view_func=self.route_info)
        self.app.add_url_rule("/api/action/moveto", view_func=self.route_action_moveto)
        self.app.add_url_rule("/api/action/search", view_func=self.route_action_search)
        self.app.add_url_rule("/api/setup/run_clustering", methods=["POST"], view_func=self.route_run_clustering)
        self.app.add_url_rule("/api/setup/confirm_clustering", methods=["POST"], view_func=self.route_confirm_clustering)
        self.app.add_url_rule("/api/setup/start_operation", view_func=self.route_start_operation)
        self.app.after_request(self.add_headers)

    def route_index(self):
        return send_from_directory(self.static_dir, "index.html")

    def route_info(self) -> Dict:
        drones = {}
        for drone_id, drone_state in self.drone_states.items():
            drones[drone_id] = drone_state.toJSON()  # Ensure this method returns a serializable dictionary

        ret = {
            "drones": drones,
            "hotspots": self.mission.hotspots,  # Assuming this is already serializable
            "clusters": self.mission.cluster_centres,  # Assuming this is already serializable
        }
        jsonify(ret)
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
        hotspots_location = json.loads(hotspots_location)
        self.mission.hotspots = {i: {"position": hotspots_location[i]["position"]} for i in range(len(hotspots_location))}
        cluster_centres = run_clustering(hotspots_location)
        return cluster_centres
    
    def route_confirm_clustering(self) -> bool:
        """"Update mission clusters centres"""
        data = request.form.to_dict()
        confirmed_clusters = json.loads(data.get("clusters", None))
        self.mission.cluster_centres = confirmed_clusters
        self.mission.cluster_centres_to_explore += confirmed_clusters
        return {}, 200
    
    def route_start_operation(self):
        """Run assignment on drones in drone state, cluster centers and command drones to search sector"""

        assignments = self.assigner.fit(self.mission.cluster_centres_to_explore, self.drone_states)
        for drone_id, cluster in assignments.items():
            command_tup = (drone_id, DroneCommand_SEARCH_SECTOR(LatLon(cluster["position"]["lat"], cluster["position"]["lng"]), None))
            self.commands.put_nowait(command_tup)
        return {}, 200        

    def add_headers(self, response: Response):
        response.headers.add("Content-Type", "application/json")
        response.headers.add("Access-Control-Allow-Methods", "PUT, GET ,POST, DELETE, OPTIONS")
        response.headers.add("Access-Control-Allow-Origin", "*")
        response.status = 200
        return response
    
    def run(self):
        self.app.run(debug=True, use_reloader=False)