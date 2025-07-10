from CameraBase import CameraBase
import logging
import cv2
import sys

logging.basicConfig(level = logging.DEBUG,
    format='%(asctime)s.%(msecs)03d %(levelname)s:\t%(message)s',
    datefmt='%H:%M:%S'
)


class LogitechCamera(CameraBase):
    """
    USB interface Logitech camera implementation.
    Inherits from CameraBase and configures resolution and frame rate for Logitech USB devices.
    """
    def __init__(self, source=0, width=640, height=480, fps=30):
        """
        :param source: device index, DirectShow name string, or filepath
        :param width: desired frame width
        :param height: desired frame height
        :param fps: desired frames per second
        """
        super().__init__(source)
        self.width = width
        self.height = height
        self.fps = fps

        logging.debug(f"A Logitech Camera is given at source {source}, Filming at {width}x{height} in {fps} fps")


    def open(self) -> bool:
        """
        Open the Logitech USB camera with appropriate backend.
        :return: True if opened successfully, False otherwise.
        """
        opened = super().open()

        if opened:
            logging.debug(f"The camera was initialized properly.")
            # Apply resolution and frame rate settings
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self._capture.set(cv2.CAP_PROP_FPS, self.fps)
        return opened

    def process_frame(self, frame):
        """
        Default frame processing: convert BGR to RGB.
        Override this method to apply custom processing.
        :param frame: raw BGR frame
        :return: RGB frame
        """
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    def set_property(self, prop_id, value) -> bool:
        """
        Set a custom OpenCV VideoCapture property.
        :param prop_id: cv2.CAP_PROP_* identifier
        :param value: property value
        :return: True if property set successfully
        """
        if not self.is_opened():
            raise RuntimeError("Camera is not opened")
        return self._capture.set(prop_id, value)

    def get_property(self, prop_id):
        """
        Get a custom OpenCV VideoCapture property.
        :param prop_id: cv2.CAP_PROP_* identifier
        :return: current property value
        """
        if not self.is_opened():
            raise RuntimeError("Camera is not opened")
        return self._capture.get(prop_id)


if __name__ == "__main__":
    # Enumerate and display available cameras
    print("Probing for available cameras...")

    cam = LogitechCamera(source=1, width=640, height=480, fps=30)
    try:
        with cam:
            logging.debug(f"Camera opened: {cam.is_opened()}")
            logging.debug(f"Resolution: {cam.get_property(cv2.CAP_PROP_FRAME_WIDTH)}x{cam.get_property(cv2.CAP_PROP_FRAME_HEIGHT)}")
            logging.debug(f"FPS: {cam.get_property(cv2.CAP_PROP_FPS)}")
            logging.debug("Press 'q' to quit.")
            while True:
                frame = cam.read()
                processed = cam.process_frame(frame)
                cv2.imshow("LogitechCamera Test", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except Exception as e:
        print(f"Error: {e}")
    finally:
        cam.close()
        cv2.destroyAllWindows()