"""
sim_generator:
Generates a simulation for the sensors.

A simulated sensor must first:
1. Initialise a SimulationMap, either directly or from a mapfile (.simmap).
2. Call the `get_signals_at()` method, passing its location in to get the signals at that point.
"""
import struct
from random import randbytes, uniform
from typing import Dict, List
from math import log10
from pathlib import Path
from maplib import LatLon, PositionXY

MEASURED_POWER = -30 # RSSI at 1 metre
ENV_FACTOR = 2.4     # Value between 2 and 4
DEFAULT_LAT = 0.0
DEFAULT_LON = 0.0

SIM_MAP_DIR = None
if Path.cwd().stem != "drone_ws":
    if Path.cwd().parent.stem == "drone_ws":
        SIM_MAP_DIR = Path.cwd().parent.joinpath("src/sensor_node/simmaps")
    else:
        print("Could not locate project root.")
        exit(1)
else:
    SIM_MAP_DIR = Path.cwd().joinpath("src/sensor_node/simmaps")

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
        print(f"(0, 0) is at LatLon: {self.start_latlon}")
        plt.grid(True)
        plt.xlim((-self.max_lat_offset, self.max_lat_offset))
        plt.ylim((-self.max_lon_offset, self.max_lon_offset))
        for device in self.devices:
            pos = device.position.toXY(self.start_latlon)
            plt.plot(pos.x, pos.y, 'bo')
            plt.annotate(device.mac, (pos.x, pos.y))
        plt.show()

    def to_file(self, name: str):
        p = SIM_MAP_DIR.joinpath(name + ".simmap")
        with p.open("wb") as fp:
            data = struct.pack("!ddddi", self.start_latlon.lat, self.start_latlon.lon, self.max_lat_offset, self.max_lon_offset, len(self.devices))
            for device in self.devices:
                data += struct.pack("!35sdd", device.mac.encode("ascii"), device.position.lat, device.position.lon)
            fp.write(data)

    @staticmethod
    def from_file(name: str) -> 'SimulationMap':
        p = SIM_MAP_DIR.joinpath(name)
        if p.suffix != ".simmap":
            print("Expected a `.simmap` file.")
            exit(1)
        if not p.is_file():
            print(f"{name} is not a file.")
            exit(1)
        
        with p.open("rb") as fp:
            data = fp.read()
            start_lat, start_lon, max_lat_offset, max_lon_offset, device_count = struct.unpack("!ddddi", data[0:36])
            start_latlon = LatLon(float(start_lat), float(start_lon))
            map = SimulationMap(0, start_latlon, max_lat_offset, max_lon_offset)

            iter = 36

            for i in range(device_count):
                mac_b, pos_lat, pos_lon = struct.unpack("!35sdd", data[iter:iter+51])
                iter += 51
                device = Device(mac_b.decode("ascii"), LatLon(float(pos_lat), float(pos_lon)))
                map.devices.append(device)
            return map

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
        device_count = int(argv[1])
        max_lat_offset = float(argv[2])
        max_lon_offset = float(argv[3])

        # Extract env variables
        lat = float(environ.get("PX4_HOME_LAT", DEFAULT_LAT))
        lon = float(environ.get("PX4_HOME_LON", DEFAULT_LON))

        map = SimulationMap(device_count, LatLon(lat, lon), max_lat_offset, max_lon_offset)
        filename = datetime.now().strftime(f"{device_count}-%Y%m%d-%H%M%S")
        map.to_file(filename)
        print(f"Exported to {filename}.simmap.")
        map.render()
    else:
        print("Usage: ./sim_generator [file]: View an existing `.simmap` file.\n       ./sim_generator [device_count] [max_lat_offset] [max_lon_offset]: Generate a new `.simmap` file.")
