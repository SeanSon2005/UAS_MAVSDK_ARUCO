from abc import ABC, abstractmethod

class Camera(ABC):
    @abstractmethod
    def start(self):
        """
        Starts the camera.
        """
        pass

    @abstractmethod
    def stop(self):
        """
        Stops the camera.
        """
        pass
    
    @abstractmethod
    def capture_frame(self):
        """
        Returns a frame from the camera.
        """
        pass