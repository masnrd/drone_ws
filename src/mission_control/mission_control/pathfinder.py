"""
pathfinder:
Pathfinding algorithm to simulate drone movement for the mission control
"""
import h3
import numpy as np
from enum import IntEnum
from abc import ABC, abstractmethod
from typing import Tuple, Dict, NewType, Union
from copy import deepcopy
from .maplib import LatLon

DEFAULT_MAX_STEPS = 30   # Maximum steps for pathfinding
DEFAULT_RESOLUTION = 14  # Default resolution of H3 map
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

def hexdist(a: str, b: str) -> float:
    """ Computes Vincenty distance between two hexes, where `a` and `b` are H3 indices. """
    lat1, lon1 = h3.h3_to_geo(a)
    lat2, lon2 = h3.h3_to_geo(b)
    ll1 = LatLon(lat1, lon1)
    ll2 = LatLon(lat2, lon2)
    return ll1.distFromPoint(ll2)


class PathfinderType(IntEnum):
    SPIRAL = 0
    BAYESIAN = 1


class PathfinderState:
    """ Pathfinding state utilised by the drone. """
    def __init__(self, start_pos: LatLon, prob_map: ProbabilityMap = None, pathfinder_type: PathfinderType = PathfinderType.SPIRAL):
        start_tup = (start_pos.lat, start_pos.lon)
        if pathfinder_type == PathfinderType.SPIRAL:
            self._pathfinder = OutwardSpiralPathFinder(DEFAULT_RESOLUTION, start_tup)
        else:
            raise NotImplementedError("TODO: Port over Bayesian search")

        if prob_map is None:
            prob_map = init_empty_prob_map(start_pos, N_RINGS_CLUSTER)

        self._prob_map = prob_map
        self.max_step = 30
        self.step_count = 0
        self.simulated_path = self.get_simulated_path(start_pos)

    def get_next_waypoint(self, cur_pos: LatLon) -> Union[LatLon, None]:
        if cur_pos is None:
            raise RuntimeError("get_next_waypoint: cur_pos is None")
        self.step_count += 1
        if (self.step_count > self.max_step):
            return None

        cur_tup = (cur_pos.lat, cur_pos.lon)
        next_tup = self._pathfinder.find_next_step(cur_tup, self._prob_map)
        if next_tup is None:
            return None
        return LatLon(next_tup[0], next_tup[1])
    
    def get_simulated_path(self, cur_pos: LatLon) -> Dict[int, Dict]:
        sim_map = deepcopy(self._prob_map)
        step = 0
        simulated_path = dict()

        while step < self.max_step:
            cur_tup = (cur_pos.lat, cur_pos.lon)
            next_tup = self._pathfinder.find_next_step(cur_tup, sim_map)
            cur_pos = LatLon(next_tup[0], next_tup[1])
            simulated_path[step] = cur_pos.to_dict()
            step += 1
        
        return simulated_path
    
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
    def find_next_step(self, current_position: Tuple[float, float], prob_map: np.ndarray) -> Union[Tuple[float, float], None]:
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
    def __init__(self, res: int, centre: Tuple[float, float]):
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
    def find_next_step(self, current_position: Tuple[float, float], prob_map: ProbabilityMap) -> Tuple[float, float]:
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
