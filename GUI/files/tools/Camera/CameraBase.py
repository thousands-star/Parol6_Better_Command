from abc import ABC, abstractmethod
import cv2
import platform
import logging

# Setting Up format for logging
logging.basicConfig(level=logging.DEBUG,
    format='%(asctime)s.%(msecs)03d %(levelname)s:\t%(message)s',
    datefmt='%H:%M:%S'
)

# Abstractize Base Class for a camera.
# TODO: If there is any different camera, they should also inherit from this Camera Base.
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
        self._source = source        # 摄像头来源（编号或视频文件）
        self._capture = None         # OpenCV VideoCapture 对象

    def open(self) -> bool:
        """
        Open the camera device.
        :return: True if opened successfully, False otherwise.
        """
        my_os = platform.system()
        if self._capture is None:
            # 根据操作系统使用更合适的后端（Windows 用 DirectShow 更快）
            if my_os == "Windows":
                try:
                    self._capture = cv2.VideoCapture(self._source, cv2.CAP_DSHOW)
                except:
                    self._capture = cv2.VideoCapture(self._source)
            else:
                self._capture = cv2.VideoCapture(self._source)
        return self._capture.isOpened()  # 返回是否成功打开摄像头

    def close(self):
        """
        Release the camera device.
        """
        if self._capture is not None:
            self._capture.release()  # 释放摄像头资源
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
        :return: frame (image array)
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
        pass  # 必须在子类中实现

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



# Logitech Camera as a child class.
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
            # 设置摄像头分辨率和帧率
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

# 测试入口：打开摄像头并显示预览窗口
if __name__ == "__main__":
    print("Probing for available cameras...")

    cam = LogitechCamera(source=1, width=640, height=480, fps=30)
    try:
        with cam:
            logging.debug(f"Camera opened: {cam.is_opened()}")
            logging.debug(f"Resolution: {cam.get_property(cv2.CAP_PROP_FRAME_WIDTH)}x{cam.get_property(cv2.CAP_PROP_FRAME_HEIGHT)}")
            logging.debug(f"FPS: {cam.get_property(cv2.CAP_PROP_FPS)}")
            logging.debug("Press 'q' to quit.")
            while True:
                frame = cam.read()  # 从摄像头读取一帧
                processed = cam.process_frame(frame)  # 默认转换为 RGB
                # 注意：cv2.imshow() 只能显示 BGR，因此要转回 BGR
                cv2.imshow("LogitechCamera Test", cv2.cvtColor(processed, cv2.COLOR_RGB2BGR))
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except Exception as e:
        print(f"Error: {e}")
    finally:
        cam.close()
        cv2.destroyAllWindows()
