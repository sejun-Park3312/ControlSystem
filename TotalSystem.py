from Vision import Vision
from Control import Control
from Arduino import Arduino
import time
import cv2
import threading

class TotalSystem:
    def __init__(self):

        self.VS = Vision()
        self.CT = Control()
        self.AD = Arduino()

        self.lock = threading.Lock()
        self.Running = True


    def Start(self):
        # Thread
        Thread_Vision = threading.Thread(target=self.VS.Tracking, daemon=True)
        Thread_Vision.start()

        # Thread_Arduino = threading.Thread(target=self.AD.ManualPWM, daemon=True)
        # Thread_Arduino.start()

        StartTime = time.time()
        while self.Running:

            if cv2.waitKey(1) == 27:  # ESC
                self.VS.Running = False
                self.AD.Running = False
                self.Running = False
                print("Test Stopped!")
                break

            time.sleep(self.CT.SamplingTime)
            with self.lock:
                self.CT.Z_Target = self.VS.Z

            PWM = self.CT.Get_PWM()
            self.AD.Send_PWM(PWM)
            print(PWM)


        print("TotalSystem Ended!")



