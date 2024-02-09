from typing import Dict, List

from drone_utils import DroneMode
from .interface import Assigner

# DBScan ClusterFinder


class SimpleQueueAssigner(Assigner):
    """
    SimpleQueueAssigner that uses round robin strategy and takes in the drones states and queue 
    """

    def __init__(self):
        pass

    def fit(self, cluster_centres_to_explore: List, drone_states: Dict) -> Dict:
        """
        Run the assignment algorithm
        """
        assignment = {}
        available_drones = set()

        for drone_id, drone_state in drone_states.items():
            if drone_state not in [DroneMode.DISCONNECTED]: # TODO: Update based on all the conditions that do not allow for assignment
                available_drones.add(drone_id)

        for i in range(len(cluster_centres_to_explore)):
            if len(available_drones) > 0:
                available_drone_id = available_drones.pop()
                assignment[available_drone_id] = cluster_centres_to_explore[i]
            else:
                break
        return assignment

