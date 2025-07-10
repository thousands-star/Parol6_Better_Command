import cv2
import time

def take_logitech_picture(index=1):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)  # DSHOW 更快初始化（仅限 Windows）

    if not cap.isOpened():
        print(f"❌ Camera index {index} not available.")
        return

    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    ret, frame = cap.read()
    if ret:
        cv2.imwrite("logitech_capture.jpg", frame)
        print("✅ Image saved to logitech_capture.jpg")
    else:
        print("❌ Failed to capture image.")

    cap.release()

if __name__ == "__main__":
    start = time.perf_counter()
    take_logitech_picture(index=1)
    end = time.perf_counter()
    print(f"📸 Took {end - start:.4f} seconds.")
