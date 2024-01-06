"""
maplib:
Contains functions for converting between LatLon values and 
(x,y) coordinates (based on metres from a reference point)
"""

from dataclasses import dataclass
from math import sqrt, atan2, asin, cos, sin, radians

EARTH_RADIUS = 6378.137 * 1000 # Earth's radius in metres

@dataclass
class LatLon:
    """ Point, defined by WGS84 latitude and longitude. Code adapted from http://www.movable-type.co.uk/scripts/latlong.html. """
    lat: float
    lon: float

    def distFromPoint(self, refPt: 'LatLon') -> float:
        """ Returns the Haversine distance from another point. """
        lat1 = radians(refPt.lat)
        lat2 = radians(self.lat)
        dLat = radians(self.lat - refPt.lat)
        dLon = radians(self.lon - refPt.lon)
        h_lat = (1 - cos(dLat))/2
        h_lon = (1 - cos(dLon))/2
        return asin(sqrt(h_lat + (cos(lat1)*cos(lat2)*h_lon))) * (2 * EARTH_RADIUS)

    def toXY(self, refPt: 'LatLon') -> 'PositionXY':
        """ Converts from WGS84 coordinates (lat, lon) to a position vector relative to a reference point (x, y, refPt). """
        # 1. find the bearing: http://www.movable-type.co.uk/scripts/latlong.html
        lat1 = radians(refPt.lat)
        lat2 = radians(self.lat)
        dLon = radians(self.lon - refPt.lon)
        y_temp = sin(dLon) * cos(lat2)
        x_temp = (cos(lat1) * sin(lat2)) - (sin(lat1)*cos(lat2)*cos(dLon))
        bearing = atan2(y_temp, x_temp) # in radians

        # 2. find the distance (haversine)
        d = self.distFromPoint(refPt)

        # 3. calc vector
        new_x = d*cos(bearing)
        new_y = d*sin(bearing)
        return PositionXY(new_x, new_y, refPt)

@dataclass
class PositionXY:
    x: float
    y: float
    refPt: LatLon