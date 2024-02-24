from typing import Dict
from datetime import datetime
from mc_interface_msgs.msg import Detected
from .drone_utils import DroneId
from .maplib import LatLon

class DetectedEntity:
    def __init__(self, drone_id: DroneId, coords: LatLon, time_found: datetime):
        self.drone_id = drone_id
        self.coords = coords
        self.time_found = time_found

    def to_dict(self) -> Dict:
        return {
            "drone_id": self.drone_id,
            "coordinates": {
                "lat": self.coords.lat,
                "lon": self.coords.lon,
            },
            "time_found": {self.time_found.isoformat()}
        }
    
    @staticmethod
    def from_message(detected_msg: Detected) -> 'DetectedEntity':
        """ Converts from a ROS2 MC Detected message into a DetectedEntity instance """
        drone_id = DroneId(detected_msg.drone_id)
        coords = LatLon(detected_msg.lat, detected_msg.lon)
        time_found = datetime.fromtimestamp(detected_msg.timestamp)

        return DetectedEntity(drone_id, coords, time_found)
    
    def to_message(self) -> Detected:
        """ Converts from a DetectedEntity instance into a ROS2 MC Detected message """
        drone_id = int(self.drone_id)
        lat = float(self.coords.lat)
        lon = float(self.coords.lon)
        timestamp = int(self.time_found.timestamp())
        return Detected(drone_id, lat, lon, timestamp)
