from abc import ABC, abstractmethod
from typing import Dict, List


class Assigner(ABC):
    """Assigner Object"""
    def __init__(self):
        pass

    @abstractmethod
    def fit(self, cluster_centres_to_explore: List, drone_states: Dict) -> Dict:
        """
        Run the Assignment Algorithm and return a dictionary
        """
        return None
