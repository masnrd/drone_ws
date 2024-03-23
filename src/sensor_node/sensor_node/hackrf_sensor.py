from abc import ABC, abstractmethod
from time import sleep
from .maplib import LatLon
from .sim_generator import SimulationMap

RSSI_THRESHOLD = -90  # The threshold for a signal to be "detected". Below this, signals are undetectable.
SCAN_INTERVAL = 0.5   # The minimal interval between scans

class SensorModule(ABC):
    @abstractmethod
    def scan(self, channel: int = 8, time_frame: float = 2, threshold: int = 0) -> bool:
        """
        Abstract method to scan for signals within a given time frame and threshold.

        Args:
            channel: The channel number to scan.
            time_frame: The duration for which to scan in seconds
            threshold: The threshold value for deciding the result of the scan.

        Returns:
            A boolean value indicating whether the scan criteria are met.
        """
        pass

class SimulatedSensorModule(SensorModule):
    def __init__(self, map_file: str):
        self._simmap = SimulationMap.from_file(map_file)
        self._curpos = None

    def _update_position(self, new_pos: LatLon):
        self._curpos = new_pos

    def scan(self, channel: int = 8, time_frame: float = 2, threshold: int = 0) -> bool:
        scans = int(max(1, time_frame // SCAN_INTERVAL))
        detected = False
        for i in range(scans):
            if not detected:
                if self._curpos is not None:
                    signals = self._simmap.get_signals_at(self._curpos)
                    for device_mac, rssi in signals.items():
                        if detected:
                            break
                        try:
                            rssi = float(rssi)
                        except ValueError:
                            continue
                        if rssi > RSSI_THRESHOLD:
                            detected = True
            sleep(SCAN_INTERVAL)
        return detected
