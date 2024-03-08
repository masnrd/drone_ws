"""
maplib:
Contains functions for converting between LatLon values and 
(x,y) coordinates (based on metres from a reference point)
"""

from dataclasses import dataclass
from pymap3d import ned2geodetic, geodetic2ned
from pymap3d.ellipsoid import Ellipsoid
from pymap3d.vincenty import vdist

EARTH_RADIUS = 6378.137 * 1000 # Earth's radius in metres
DEFAULT_ALTITUDE = 5.0 #TODO: have the px4 report the current altitude for more accurate computation
ELLIPSOID = Ellipsoid.from_name("wgs84")

@dataclass
class LatLon:
    """ A geodesic point """
    lat: float
    lon: float

    def distFromPoint(self, refPt: 'LatLon') -> float:
        """ Returns the Vincenty distance from a reference point. """
        return vdist(self.lat, self.lon, refPt.lat, refPt.lon)[0]

    def toXY(self, refPt: 'LatLon') -> 'PositionXY':
        """ Converts from WGS84 coordinates (lat, lon) to a position vector relative to a reference point (x, y, refPt). """
        ned = geodetic2ned(self.lat, self.lon, DEFAULT_ALTITUDE, refPt.lat, refPt.lon, DEFAULT_ALTITUDE, ell=ELLIPSOID)
        return PositionXY(ned[0], ned[1], refPt)
    
    def __repr__(self) -> str:
        return f"({self.lat}, {self.lon})"

@dataclass
class PositionXY:
    x: float
    y: float
    refPt: LatLon
    
    def toLatLon(self) -> LatLon:
        """ Converts from a position vector relative to a reference point (x, y, refPt) to WGS84 coordinates (lat, lon) """
        geodetic = ned2geodetic(self.x, self.y, -DEFAULT_ALTITUDE, self.refPt.lat, self.refPt.lon, DEFAULT_ALTITUDE, ell=ELLIPSOID)
        return LatLon(geodetic[0], geodetic[1])
