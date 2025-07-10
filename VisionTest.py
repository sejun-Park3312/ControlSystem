from Vision import Vision
import cv2
import time
import keyboard
import threading

VS = Vision()
vision_thread = threading.Thread(target=VS.Tracking, daemon = True)
vision_thread.start()

while VS.Running:
    if cv2.waitKey(1) == 27:  # ESC
        VS.Running = False
        print("Test Stopped!")
        break
