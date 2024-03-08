from abc import ABC, abstractmethod

class SensorModule(ABC):
    @abstractmethod
    def scan(self) -> bool:
        """ Returns if a scan resulted in a detection or not. """
        pass