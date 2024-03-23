import json
from enum import IntEnum
from datetime import datetime
from typing import List
from .detection_utils import DetectedEntity

class MissionStage(IntEnum):
    """ Current mode reported of mission """
    SETUP = 1 # Establishing drone resources and location of base
    CLUSTERING = 2 # Input hotspots and return hotspots
    PATHFINDING = 3 # Drones are independently pathfinding 
    COMPLETE = 4 # Mission is complete

class Mission:
    """
    State of Mission
    """
    def __init__(self):
        self.stage = MissionStage.SETUP
        self.duration = datetime.now()
        self.hotspots = set()
        self.cluster_centres = {}
        self.cluster_centres_to_explore = [] # Queue of clusters to explore
        self.detected: List[DetectedEntity] = []

class SetEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, set):
            return list(obj)
        return json.JSONEncoder.default(self, obj)