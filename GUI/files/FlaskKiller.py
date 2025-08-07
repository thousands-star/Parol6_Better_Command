import threading
import time
import requests

class FlaskShutdownThread(threading.Thread):
    def __init__(self, stop_event: threading.Event, delay_s: float = 0.5):
        super().__init__(daemon=True)
        self.stop_event = stop_event
        self.delay_s = delay_s

    def run(self):
        print("[FlaskShutdownThread] Monitoring stop_event...")
        while not self.stop_event.is_set():
            time.sleep(self.delay_s)

        print("[FlaskShutdownThread] stop_event detected. Sending shutdown request...")
        try:
            requests.post("http://localhost:8000/api/exit")
        except Exception as e:
            print(f"[FlaskShutdownThread] Shutdown failed: {e}")
