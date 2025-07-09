import serial, logging
class SerialCommError(Exception):
    """Raised when serial port cannot be opened or used."""
    pass

class SerialComm:
    def __init__(self, port: str | int, baudrate: int = 3_000_000, timeout: float = 0.01):
        self.port = str(port)
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        # 在 __init__ 里调用 open，让初始化时就能报错
        self.open()

    def open(self):
        """
        Attempt to open the serial port, closing any previous handle first.
        Raises:
            SerialCommError: if port cannot be opened.
        """
        # 先关掉已有的句柄
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                logging.info(f"[SerialComm] Closed existing serial handle on {self.port}")
        except Exception:
            pass

        # 再重新打开
        try:
            self.ser = serial.Serial(port=self.port,
                                     baudrate=self.baudrate,
                                     timeout=self.timeout)
            logging.info(f"[SerialComm] Opened {self.port}@{self.baudrate}")
        except Exception as e:
            logging.error(f"[SerialComm] Failed to open {self.port}: {e}")
            # 不再回退，直接抛给上层
            raise SerialCommError(f"Cannot open serial port {self.port}: {e}")

    def is_open(self) -> bool:
        return bool(self.ser and self.ser.is_open)

    def write(self, chunks: list[bytes]):
        if not self.is_open():
            raise SerialCommError(f"Cannot write: port {self.port} is not open")
        for b in chunks:
            try:
                self.ser.write(b)
            except Exception as e:
                logging.warning(f"[SerialComm] write failed: {e}")

    def read(self, max_bytes: int = 255) -> list[bytes]:
        if not self.is_open():
            raise SerialCommError(f"Cannot read: port {self.port} is not open")
        out = []
        try:
            while self.ser.in_waiting and len(out) < max_bytes:
                out.append(self.ser.read(1))
        except Exception as e:
            logging.warning(f"[SerialComm] read error: {e}")
        return out

    def close(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                logging.info(f"[SerialComm] Closed {self.port}")
        except Exception as e:
            logging.warning(f"[SerialComm] close failed: {e}")

#!/usr/bin/env python3
# test_serial_comm.py

import platform
import time

def main():
    # 自动选一个常见的测试端口
    if platform.system() == "Windows":
        test_port = "COM3"
    else:
        test_port = "/dev/ttyACM0"

    print(f"[Test] 使用端口：{test_port}")

    # 初始化并打开
    ser = SerialComm(port=test_port, baudrate=115200, timeout=0.1)
    print(f"[Test] 打开后 is_open() → {ser.is_open()}")

    # 小停一下，确保驱动就绪
    time.sleep(0.5)

    # 如果打开成功，就尝试写入和读取
    if ser.is_open():
        test_data = [b'H', b'i', b'\n']
        print(f"[Test] 写入：{test_data}")
        ser.write(test_data)

        # 等待设备回环或者其他设备响应
        time.sleep(0.2)

        received = ser.read(max_bytes=64)
        print(f"[Test] 读到：{received}")
    else:
        print("[Test] 串口未打开，跳过读写测试")

    # 关闭
    ser.close()
    print(f"[Test] 关闭后 is_open() → {ser.is_open()}")

if __name__ == "__main__":
    main()

