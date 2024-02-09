from typing import Tuple
from .maplib import LatLon

class Point:
    def __init__(self, id, latlon_tup: Tuple[float, float]):
        """ Initialises a point for clustering, taking the latitude and longitude as a tuple. """
        self.id = id
        self.latlon = LatLon(latlon_tup[0], latlon_tup[1])
        self.visited = False
        self.cluster = None
        self.is_noise = False

    @property
    def coordinates(self) -> Tuple[float, float]:
        """ The current latitude and longitude, as a tuple. """
        return (self.latlon.lat, self.latlon.lon)

    def distance(self, other_point: 'Point'):
        """
        Compute the distance between this point and another point in kilometers
        """
        return self.latlon.distFromPoint(other_point.latlon) / 1000

    def __str__(self):
        return f'''Point {self.id}[{str(self.coordinates)}, cluster:{self.cluster}]'''
