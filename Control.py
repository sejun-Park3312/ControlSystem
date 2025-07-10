import threading
import numpy as np
from simple_pid import PID
from BasicMagnetFuns import BasicMagnetFuns
import time

class Control:
    def __init__(self):
        # Basic Magnet Functions
        self.BF = BasicMagnetFuns()

        # Array
        self.C_Points, self.C_Angles, self.M_Points, self.M_Angles = self.Ready()

        # DipoleMoment Magnitude
        self.Ms = 2
        self.Mc = 0.92 # NA
        self.Mt = 0.0265

        # Mechanical Properties
        self.F_Buoyance = 0.005146777750500
        self.Weight = 0.006776951342543
        self.I_Max = 1.5

        # Distance Offsets
        self.Z_Offset = 100 / 1000 # system(센터 코일 높이)과 Target 사이 Reference 거리
        self.Reference = 0 # 추종값

        # PID
        self.SamplingTime = 25 / 1000
        self.PID_Gain =1e-1
        self.pid = PID(Kp=self.PID_Gain, Kd=self.PID_Gain/10, Ki=0, setpoint = 0)
        self.pid.sample_time = self.SamplingTime

        # Threading
        self.lock = threading.Lock()
        self.Running = True
        self.PWM = 0


    def Ready(self):
        # Coil/Magent Array
        Data = np.load("Array_Data/Data.npz")
        self.C_Points = Data['C_Points']
        self.C_Angles = Data['C_Angles']
        self.M_Points = Data['M_Points']
        self.M_Angles = Data['M_Angles']

        return self.C_Points, self.C_Angles, self.M_Points, self.M_Angles





        ZdTIT_Data = []
        while self.running:
            time.sleep(self.Ts_Control)
            with self.lock:
                Z = self.Z
                dT = self.dT
            F_need = self.pid(Z, dt = dT)
            I_input = (self.Weight + F_need - self.MF.MagnetArray_Force(Z + self.Ref_Movement) - self.F_Buoyance) / self.MF.CoilArray_ACoeff(Z + self.Ref_Movement)
            self.I_discrete = float(np.round(np.clip(I_input, 0, self.I_Max) / 0.02) * 0.02)

            ZdTIT_Data.append((Z, dT, self.I_discrete, time.time()))

        with open('Results_Data/ZdTIT_Data.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Z', 'dT', 'I', 'time'])
            for row in ZdTIT_Data:
                writer.writerow(row)

    def Arduino(self, I):

        for i in range(8):
            self.pwm_list[i].write(I/self.I_Max)


    def Start(self):

        # Vision Thread
        vision_thread = threading.Thread(target=self.Vision)
        vision_thread.daemon = True
        vision_thread.start()

        # Control Thread
        control_thread = threading.Thread(target=self.Control)
        control_thread.daemon = True
        control_thread.start()

        # Arduino Start
        self.it.start()
        time.sleep(1)
        for i in range(8):
            self.brk_list[i].write(0)  # 브레이크 해제

        # Start(Esc 누르면 탈출)
        print(f"⏹️ 드라이버 {i} 시작!!!\n")
        Curr_Time = 0
        try:
            while self.running:
                time.sleep(self.Ts_Control)
                self.Arduino(self.I_discrete)
                if time.time() - Curr_Time > 1:
                    print(f"Z: {self.Z*1000:.2f} mm, A: {self.I_discrete:.2f} A")
                    Curr_Time = time.time()

                if cv2.waitKey(1) == 27:  # ESC
                    self.running = False
                    break

        except KeyboardInterrupt:
            self.running = False  # 종료 플래그 내려서 두 스레드 종료
            vision_thread.join()
            control_thread.join()

        for i in range(8):
            self.pwm_list[i].write(0)  # PWM OFF
            self.brk_list[i].write(1)  # 브레이크 ON

        print(f"⏹️ 드라이버 {i} 정지됨!!!\n")
        time.sleep(1)  # 다음 드라이버 전환 전 잠깐 대기

        # 종료 처리
        self.board.exit()