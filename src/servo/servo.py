from abc import ABC, abstractmethod

class Servo(ABC):
    @abstractmethod
    def angle_to_steps(self, angle: float) -> float:
        pass

    @abstractmethod
    def move(self, angle: float, label: str):
        pass
    
    @abstractmethod
    def close(self):
        pass
