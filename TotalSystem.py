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

        self.lock = threading.Lock()
        self.Running = True

        self.Data = RealTimeData_Recorder()
        self.DataName = "ControlData"
        self.Data.DefineData(self.DataName, ["Z_Error", "Z_System", "Z_Target", "PWM"])


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
                PWM = self.CT.Get_PWM()
                self.Data.AppendData(self.DataName, [(self.CT.Z_Reference - (self.CT.Z_System - self.CT.Z_Target)),
                                                     self.CT.Z_System, self.CT.Z_Target, PWM])

            self.AD.Send_PWM(PWM)
            print(PWM)

        self.AD.Disconnect()
        print("TotalSystem Ended!")


    def SaveResults(self):
        print("Saving Results...")
        # Align Vision Timestamp to Reference
        self.Data[self.DataName]["Time"]["StartTime"] = self.VS.Data[self.VS.DataName]["Time"]["StartTime"]
        # Save Data
        self.VS.Data.SaveData(self.VS.DataName, self.VS.DataName)
        self.VS.Data.SaveData(self.DataName, self.DataName)
