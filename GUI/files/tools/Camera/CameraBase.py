from abc import ABC, abstractmethod
import cv2
import platform

class CameraBase(ABC):
    """
    Abstract base class for camera operations.
    Provides methods to open, close, and capture frames from a camera device.
    """
    def __init__(self, source=0):
        """
        Initialize with camera source (index or video file path).
        :param source: device index or video file path
        """
        self._source = source
        self._capture = None

    def open(self) -> bool:
        """
        Open the camera device.
        :return: True if opened successfully, False otherwise.
        """
        my_os = platform.system()
        if self._capture is None:
            print("self._source")
            if my_os == "Windows":
                try:
                    self._capture = cv2.VideoCapture(self._source, cv2.CAP_DSHOW)
                except:
                    self._capture = cv2.VideoCapture(self._source)
            else:
                self._capture = cv2.VideoCapture(self._source)
        return self._capture.isOpened()

    def close(self):
        """
        Release the camera device.
        """
        if self._capture is not None:
            self._capture.release()
            self._capture = None

    def is_opened(self) -> bool:
        """
        Check if the camera is opened.
        :return: True if open, False otherwise.
        """
        return self._capture is not None and self._capture.isOpened()

    def read(self):
        """
        Read a frame from the camera.
        :return: (ret, frame) where ret is a boolean and frame is the image.
        """
        if not self.is_opened():
            raise RuntimeError("Camera is not opened")
        ret, frame = self._capture.read()
        if not ret:
            raise RuntimeError("Failed to read frame from camera")
        return frame

    @abstractmethod
    def process_frame(self, frame):
        """
        Process a captured frame. Must be implemented by subclasses.
        :param frame: raw frame from read()
        :return: processed frame or result
        """
        pass

    def __enter__(self):
        """
        Context manager entry: open camera.
        """
        if not self.open():
            raise RuntimeError(f"Unable to open camera source: {self._source}")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Context manager exit: release camera.
        """
        self.close()
