import cv2
import threading
import multiprocessing
from multiprocessing import Process, Manager, set_start_method
from tools.Camera.CameraBase import LogitechCamera
from tkinter import Tk, Label
from PIL import Image, ImageTk
from Vision import camera_capture_thread, tag_detector_thread, overlay_thread


def Camera_process(frame_q, display_q, stop_event):
    tag_list = []
    tag_lock = threading.Lock()
    cam = LogitechCamera(source=1, width=640, height=480, fps=30)
    t1 = threading.Thread(target=camera_capture_thread, args=(cam, frame_q, stop_event), daemon=True)
    t2 = threading.Thread(target=tag_detector_thread, args=(frame_q, tag_list, tag_lock, stop_event), daemon=True)
    t3 = threading.Thread(target=overlay_thread, args=(frame_q, display_q, tag_list, tag_lock, stop_event), daemon=True)

    t1.start()
    t2.start()
    t3.start()
    t1.join()
    t2.join()
    t3.join()

# ---------------- GUI Logic ----------------
class MinimalGUI:
    def __init__(self, root, display_q, stop_event):
        self.root = root
        self.display_q = display_q
        self.stop_event = stop_event
        self.label = Label(root)
        self.label.pack()
        self.update_loop()

    def update_loop(self):
        if not self.display_q.empty():
            frame = self.display_q.get()
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            imgtk = ImageTk.PhotoImage(image=img)
            self.label.configure(image=imgtk)
            self.label.image = imgtk
        if not self.stop_event.is_set():
            self.root.after(30, self.update_loop)


def GUI_process(display_q, stop_event):
    root = Tk()
    app = MinimalGUI(root, display_q, stop_event)
    def on_close():
        stop_event.set()
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()

# ---------------- MAIN ----------------
if __name__ == "__main__":
    set_start_method("spawn", force=True)

    manager = Manager()
    frame_q = multiprocessing.Queue(maxsize=1)
    display_q = multiprocessing.Queue(maxsize=1)
    stop_event = manager.Event()

    p_cam = Process(target=Camera_process, args=(frame_q, display_q, stop_event))
    p_gui = Process(target=GUI_process, args=(display_q, stop_event))

    p_cam.start()
    p_gui.start()

    p_gui.join()  # wait for GUI to close
    stop_event.set()  # notify camera process to exit
    p_cam.join()

    print("✅ Exited cleanly")
