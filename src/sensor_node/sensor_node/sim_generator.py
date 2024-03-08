"""
sim_generator:
Generates a simulation for the sensors.

A simulated sensor must first:
1. Initialise a SimulationMap, either directly or from a mapfile (.simmap).
2. Call the `get_signals_at()` method, passing its location in to get the signals at that point.
"""

import pickle
from random import randbytes, uniform
from maplib import LatLon, PositionXY
from typing import Dict, List
from math import log10
from pathlib import Path

MEASURED_POWER = -30 # RSSI at 1 metre
ENV_FACTOR = 2.4     # Value between 2 and 4
DEFAULT_LAT = 0.0
DEFAULT_LON = 0.0

def random_mac_address() -> str:
    return ':'.join([hex(b)[2:] for b in randbytes(12)])

def random_latlon_with_offset(offset_from: LatLon, offset_metres_x: float, offset_metres_y: float) -> LatLon:
    return PositionXY(
        uniform(-offset_metres_x, offset_metres_x),
        uniform(-offset_metres_y, offset_metres_y),
        offset_from
        ).toLatLon()

class Device:
    def __init__(self, mac: str, position: LatLon):
        self.mac = mac
        self.position = position

class SimulationMap:
    def __init__(self, num_devices: int, start_latlon: LatLon = LatLon(0.0, 0.0), max_lat_offset: float = 5.0, max_lon_offset: float = 5.0):
        """
        Initialise a new simulated map, which randomly generates the positions of simulated devices. 
        - `max_lat_offset`, `max_lon_offset` defines the maximum offset of a device's latitude and longitude from `start_latlon`. 
        """
        self.devices: List[Device] = []
        self.start_latlon = start_latlon
        self.max_lat_offset = max_lat_offset
        self.max_lon_offset = max_lon_offset
        for i in range(num_devices):
            self.devices.append(Device(
                random_mac_address(),
                random_latlon_with_offset(start_latlon, max_lat_offset, max_lon_offset)
            ))

    def get_signals_at(self, sensor_pos: LatLon) -> Dict[str, str]:
        """ Returns the signals for a simulated detector at `sensor_pos` in this simulation map. """
        signals = {}
        for device in self.devices:
            distance = abs(device.position.distFromPoint(sensor_pos))
            simulated_rssi = MEASURED_POWER - (10 * ENV_FACTOR)* log10(distance)
            #print(f"{device.mac}: Distance {distance}, simulated {simulated_rssi}")
            if simulated_rssi < -100:
                continue
            signals[device.mac] = str(simulated_rssi)
        return signals

    def render(self):
        import matplotlib.pyplot as plt
        plt.grid(True)
        plt.xlim((-self.max_lat_offset, self.max_lat_offset))
        plt.ylim((-self.max_lon_offset, self.max_lon_offset))
        for device in self.devices:
            pos = device.position.toXY(self.start_latlon)
            plt.plot(pos.x, pos.y, 'bo')
            plt.annotate(device.mac, (pos.x, pos.y))
        plt.show()
    
    def to_file(self, name: str):
        name += ".simmap"
        with open(name, 'wb') as f:
            pickle.dump(self, f, pickle.HIGHEST_PROTOCOL)

    @staticmethod
    def from_file(name: str) -> 'SimulationMap':
        p = Path(name)
        if p.suffix != ".simmap":
            print("Expected a `.simmap` file.")
            exit(1)
        if not p.is_file():
            print(f"{name} is not a file.")
            exit(1)
        try:
            with p.open("rb") as fp:
                return pickle.load(fp)
        except Exception as e:
            print(f"Error: {e}")
            exit(1)


if __name__ == "__main__":
    from sys import argv
    from os import environ
    from datetime import datetime

    map = None
    if len(argv) == 2:
        # Import from file
        map = SimulationMap.from_file(argv[1])
        map.render()
    elif len(argv) == 4:
        # Generate a map
        try:
            device_count = int(argv[1])
            max_lat_offset = float(argv[2])
            max_lon_offset = float(argv[3])

            # Extract env variables
            lat = environ.get("PX4_HOME_LAT", DEFAULT_LAT)
            lon = environ.get("PX4_HOME_LON", DEFAULT_LON)

            map = SimulationMap(device_count, LatLon(lat, lon), max_lat_offset, max_lon_offset)
            filename = datetime.now().strftime(f"{device_count}-%Y%m%d-%H%M%S")
            map.to_file(filename)
            print(f"Exported to {filename}.simmap.")
            map.render()
        except Exception as e:
            print(f"Error: {e}")
            exit(1)
    else:
        print("Usage: ./sim_generator [file]: View an existing `.simmap` file.\n       ./sim_generator [device_count] [max_lat_offset] [max_lon_offset]: Generate a new `.simmap` file.")
