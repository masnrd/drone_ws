"""
pathfinder:
Pathfinding algorithm for the drone

NOTICE for Pathfinding team:
- I renamed all instances of 'center' to 'centre', and 'centre_hexagon' to 'centre_hex'
- Currently, `None` is passed to `prob_map`, so an error could occur if the code relies on it.
"""
import h3
import numpy as np
from abc import ABC, abstractmethod
from typing import Tuple, Dict, NewType
from maplib import LatLon

DEFAULT_RESOLUTION = 12
N_RINGS_CLUSTER = 16     # Defines the number of rings in a cluster by default

# Probability Map type definition: A dictionary, where each H3 hexagon index is mapped to a specific probability.
ProbabilityMap = NewType("ProbabilityMap", Dict[str, float])

def init_empty_prob_map(centre_pos: LatLon, n_rings: int) -> ProbabilityMap:
    """
    Initialises an empty probability map.
    - `centre_pos`: Centre of the probability map.
    - `n_rings`: Number of rings around the centre hexagon.
    """
    prob_map = {}
    h3_indices = h3.k_ring(
        h3.geo_to_h3(centre_pos.lat, centre_pos.lon, DEFAULT_RESOLUTION),
        n_rings,
    )

    for h3_index in h3_indices:
        prob_map[h3_index] = 0
    
    return prob_map

class PathfinderState:
    """ Pathfinding state utilised by the drone. """
    def __init__(self, start_pos: LatLon, prob_map: ProbabilityMap = None):
        if prob_map is None:
            prob_map = init_empty_prob_map(start_pos, N_RINGS_CLUSTER)

        start_tup = (start_pos.lat, start_pos.lon)
        self._pathfinder = OutwardSpiralPathFinder(DEFAULT_RESOLUTION, start_tup)  #TODO: to be set by the caller, based on the search method defined by MC
        self._prob_map = prob_map

    def get_next_waypoint(self, cur_pos: LatLon) -> LatLon:
        cur_tup = (cur_pos.lat, cur_pos.lon)
        next_tup = self._pathfinder.find_next_step(cur_tup, self._prob_map)
        return LatLon(next_tup[0], next_tup[1])
    
    def found_signals(self, cur_pos: LatLon, signal_count: int):
        pass

class PathFinder(ABC):
    def __init__(self, res: int, centre: Tuple[float, float]):
        """
        - `res`: H3 Resolution
        - `centre`: Starting position for pathfinder, as tuple of latitude, longitude
        """
        self.res = res
        self.centre_hex = h3.geo_to_h3(centre[0], centre[1], resolution=self.res)

    @abstractmethod
    def find_next_step(self, current_position: Tuple[float, float], prob_map: np.ndarray) -> Tuple[float, float]:
        """
        Find the next step to go to.

        Args:
            current_position (Tuple[float, float]): Current coordinates lat lon
            prob_map: np.ndarray - of size 7*..7 equal to size of dimension
        Returns:
            Tuple[float, float] - Next step coordinates in lat lon.
        """
        pass

class OutwardSpiralPathFinder(PathFinder):
    def __init__(self, res: int, centre: tuple):
        super().__init__(res, centre)
        self.segment_start_ij_coord = None
        self.next_path_segment = []
        self.k_ring = 1

        centre_ij_coord = h3.experimental_h3_to_local_ij(
            self.centre_hex, self.centre_hex)
        self.next_path_segment.append(centre_ij_coord)
        self.segment_start_ij_coord = centre_ij_coord

    # Create path over an entire circle
    def ring_edge_traversal(self, repetitions, current_ij_coord, i_increment, j_increment):
        for i in range(repetitions):
            current_ij_coord = (
                current_ij_coord[0]+i_increment, current_ij_coord[1]+j_increment)
            self.next_path_segment.append(current_ij_coord)
        return current_ij_coord

    # Implementation of abstract method that returns next waypoint
    def find_next_step(self, current_position: tuple[float, float], prob_map: dict) -> tuple[float, float]:
        current_position_ij = h3.experimental_h3_to_local_ij(self.centre_hex, h3.geo_to_h3(
            current_position[0], current_position[1], resolution=self.res))

        # Waypoints are calculated based on ring
        if len(self.next_path_segment) == 1 and self.segment_start_ij_coord == current_position_ij:
            self.segment_start_ij_coord = self.ring_edge_traversal(
                1, self.segment_start_ij_coord, 0, -1)
            self.segment_start_ij_coord = self.ring_edge_traversal(
                self.k_ring-1, self.segment_start_ij_coord, 1, 0)
            self.segment_start_ij_coord = self.ring_edge_traversal(
                self.k_ring, self.segment_start_ij_coord, 1, 1)
            self.segment_start_ij_coord = self.ring_edge_traversal(
                self.k_ring, self.segment_start_ij_coord, 0, 1)
            self.segment_start_ij_coord = self.ring_edge_traversal(
                self.k_ring, self.segment_start_ij_coord, -1, 0)
            self.segment_start_ij_coord = self.ring_edge_traversal(
                self.k_ring, self.segment_start_ij_coord, -1, -1)
            self.segment_start_ij_coord = self.ring_edge_traversal(
                self.k_ring, self.segment_start_ij_coord, 0, -1)
            self.k_ring += 1

        if current_position_ij == self.next_path_segment[0]:
            self.next_path_segment.pop(0)
            return h3.h3_to_geo(h3.experimental_local_ij_to_h3(self.centre_hex, self.next_path_segment[0][0], self.next_path_segment[0][1]))
        else:
            print("Previous waypoint may not be correct")
            return None
