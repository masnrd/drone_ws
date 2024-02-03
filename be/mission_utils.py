from enum import IntEnum
from datetime import datetime

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