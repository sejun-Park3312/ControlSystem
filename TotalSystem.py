from Vision import Vision
from Control import Control
from Arduino import Arduino
from RealTimeData_Recorder import RealTimeData_Recorder
import time
import cv2
import threading

class TotalSystem:
    def __init__(self):
        self.VS = Vision()
        self.CT = Control()
        self.AD = Arduino()
        self.DR = RealTimeData_Recorder()

        self.lock = threading.Lock()
        self.Running = True


    def Ready(self):
        self.DR.DefineData("VisionData", ["X", "y", "Z"])
        self.DR.DefineData("ControlData", ["Z_Error", "Z_System", "Z_Target", "PWM"])


    def Start(self):
        # Thread
        Thread_Vision = threading.Thread(target=self.VS.Tracking, daemon=True)
        Thread_Vision.start()

        StartTime = time.time()
        while self.Running:

            cv2.waitKey(1)
            if self.VS.Running == False:
                self.AD.Running = False
                self.Running = False
                print("Test Stopped!")
                break

            time.sleep(self.CT.SamplingTime)
            with self.lock:
                self.CT.Z_Target = self.VS.Z
                self.DR.AppendData("VisionData", self.VS.XYZT_Data)

            PWM = self.CT.Get_PWM()
            self.AD.Send_PWM(PWM)
            print(PWM)

        self.AD.Disconnect()
        print("TotalSystem Ended!")

