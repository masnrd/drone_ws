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
import logging
import json
from flask import Flask, jsonify, request, send_from_directory, Response
from flask_cors import CORS
from typing import Dict, Tuple, List
from queue import Queue
from pathlib import Path
from ament_index_python import get_package_share_directory
from .maplib import LatLon
from .mission_utils import Mission
from .assigner.simplequeueassigner import SimpleQueueAssigner
from .run_clustering import run_clustering
from .drone_utils import DroneState, DroneId
from .drone_utils import DroneCommand, DroneCommand_SEARCH_SECTOR, DroneCommand_RTB, DroneCommand_MOVE_TO, DroneCommand_LAND, DroneCommand_DISCONNECT
from .detection_utils import DetectedEntity

logging.getLogger("flask_cors").level = logging.ERROR
logging.getLogger("werkzeug").level = logging.ERROR

class MCWebServer:
    def __init__(self, mission: Mission, drone_states: Dict[int, DroneState], commands: Queue[Tuple[DroneId, DroneCommand]], detected_queue: Queue[DetectedEntity]):
        self.static_dir = Path(get_package_share_directory("mission_control")).joinpath("frontend")
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
        self.detected_queue: Queue[DetectedEntity] = detected_queue
        self.assigner = SimpleQueueAssigner()

        # Set up Endpoints
        self.app.add_url_rule("/", view_func=self.route_index)
        self.app.add_url_rule("/api/info", view_func=self.route_info)
        self.app.add_url_rule("/hotspot/add", methods=["POST"], view_func=self.route_add_hotspot)
        self.app.add_url_rule("/hotspot/delete", methods=["POST"], view_func=self.route_delete_hotspot)
        self.app.add_url_rule("/api/action/moveto", view_func=self.route_action_moveto)
        self.app.add_url_rule("/api/action/search", view_func=self.route_action_search)
        self.app.add_url_rule("/api/action/rtb", view_func=self.route_action_rtb)
        self.app.add_url_rule("/api/action/land", view_func=self.route_action_land)
        self.app.add_url_rule("/api/action/disconnect", view_func=self.route_action_disconnect)
        self.app.add_url_rule("/api/setup/run_clustering", view_func=self.route_run_clustering)
        self.app.add_url_rule("/api/setup/start_operation", view_func=self.route_start_operation)
        self.app.after_request(self.add_headers)

    def drone_exists(self, drone_id: DroneId):
        return drone_id in self.drone_states.keys()

    def route_index(self):
        return send_from_directory(self.static_dir, "index.html")

    def route_info(self) -> Dict:
        drones = {}
        for drone_id, drone_state in self.drone_states.items():
            drones[drone_id] = drone_state.get_dict()

        while not self.detected_queue.empty():
            self.mission.detected.append(self.detected_queue.get())

        ret = {
            "drones": drones,
            "hotspots": list(self.mission.hotspots),         # Assuming this is already serializable
            "clusters": self.mission.cluster_centres,        # Assuming this is already serializable
            "clusters_to_explore": self.mission.cluster_centres_to_explore,
            "detected": [entity.to_dict() for entity in self.mission.detected]
        }
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
    
    def route_action_rtb(self) -> Tuple[Dict, int]:
        drone_id = request.args.get("drone_id", type=int, default=None)
        lat = request.args.get("lat", type=float, default=None)
        lon = request.args.get("lon", type=float, default=None)

        if drone_id is None:
            return {"error": "need drone_id"}, 400

        if lat is None or lon is None:
            return {"error": "need latitude/longitude parameters as float"}, 400
        
        # Place command in command queue
        command_tup = (drone_id, DroneCommand_RTB(LatLon(lat, lon), None))
        self.commands.put_nowait(command_tup)
        
        return {}, 200
    
    def route_action_land(self) -> Tuple[Dict, int]:
        drone_id = request.args.get("drone_id", type=int, default=None)

        if drone_id is None:
            return {"error": "need drone_id"}, 400
        
        # Place command in command queue
        command_tup = (drone_id, DroneCommand_LAND())
        self.commands.put_nowait(command_tup)
        
        return {}, 200
    
    def route_action_disconnect(self) -> Tuple[Dict, int]:
        drone_id = request.args.get("drone_id", type=int, default=None)

        if drone_id is None:
            return {"error": "need drone_id"}, 400
        
        # Place command in command queue
        command_tup = (drone_id, DroneCommand_DISCONNECT())
        self.commands.put_nowait(command_tup)
        
        return {}, 200
    
    def route_add_hotspot(self):
        data = request.form.to_dict()
        hotspot = json.loads(data.get("hotspot_position", None))
        self.mission.hotspots.add((hotspot["latlng"]["lat"], hotspot["latlng"]["lng"]))  # Should be a set here
        return {}, 200
    
    def route_delete_hotspot(self):
        data = request.form.to_dict()
        hotspot = json.loads(data.get("hotspot_position", None))
        try:
            self.mission.hotspots.remove((hotspot["latlng"][0], hotspot["latlng"][1]))  # Should be a set here
        except KeyError:
            return {}, 404
        return {}, 200

    def route_run_clustering(self):
        cluster_centres = run_clustering(self.mission.hotspots)
        self.mission.cluster_centres = cluster_centres
        self.mission.cluster_centres_to_explore = [cluster for cluster in cluster_centres.values()]
        return cluster_centres
    
    def route_start_operation(self):
        """ Run assignment on drones in drone state, cluster centers and command drones to search sector """
        assignments = self.assigner.fit(self.mission.cluster_centres_to_explore, self.drone_states)
        for drone_id, cluster in assignments.items():
            command_tup = (drone_id, DroneCommand_SEARCH_SECTOR(LatLon(cluster[0][0], cluster[0][1]), None))
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