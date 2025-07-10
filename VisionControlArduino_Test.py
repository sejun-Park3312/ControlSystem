from Vision import Vision
from Control import Control
from Arduino import Arduino
import cv2
import threading

VS = Vision()
CT = Control()
AD = Arduino()
lock = threading.Lock()

Thread_Vision = threading.Thread(target=VS.Tracking, daemon = True)
Thread_Arduino = threading.Thread(target=AD.ManualPWM, daemon = True)
Thread_Vision.start()
Thread_Arduino.start()

while VS.Running:
    if cv2.waitKey(1) == 27:  # ESC
        VS.Running = False
        AD.Running = False
        print("Test Stopped!")
        break

    with lock:
        VS.

