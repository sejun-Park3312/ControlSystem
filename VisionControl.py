import cv2
import threading
import numpy as np
from simple_pid import PID
from MakeFuns import MakeFuns
from pyfirmata2 import ArduinoMega, util
import time
import csv

class VisionControl:
    def __init__(self):

        print("Vision Initializaing...")
        # MakeFuns
        self.MF = MakeFuns()

        # Control Setting
        self.z_offset = 10/1000
        self.Ts_Control = 20/1000
        self.F_Buoyance = 0.005146777750500
        self.I_Max = 1.5
        self.Ref_Movement = 10/1000 # Target과 Array간 거리를 이만큼 줄이겠다
        self.PID_Gain =1e-1
        self.pid = PID(Kp=self.PID_Gain, Kd=self.PID_Gain/10, Ki=0, setpoint = 0)
        self.pid.sample_time = self.Ts_Control
        self.Weight = 0.006776951342543

        # Vision Setting
        self.Ts_Vision = 15/1000
        self.Cam1, self.Cam2, self.P1, self.P2, self.K1, self.K2, self.D1, self.D2, self.ROI_1, self.ROI_2 = self.ReadyVision()
        self.R_World2Cam1 = np.array([[1, 0, 0],
                      [0, 0, 1],
                      [0, -1, 0]], dtype=np.float64)
        self.P_World2Cam1 = np.array([[0],
                      [-200/1000],
                      [self.z_offset]], dtype=np.float64)  # 이동 벡터 (3x1)
        self.T_World2Cam1 = T = np.vstack((np.hstack((self.R_World2Cam1, self.P_World2Cam1.reshape(3,1))), [[0, 0, 0, 1]]))

        # Arduino
        self.board, self.it, self.pwm_list, self.dir_list, self.brk_list = self.ReadyArduino()

        # Threading
        self.lock = threading.Lock()
        self.Z = 0
        self.dT = 1000
        self.I_discrete = 0
        self.running = True
        self.Start_Time = 0

        print("Vision Ready!")


    def ReadyArduino(self):

        board = ArduinoMega('COM4')  # 실제 연결된 포트로 수정 필요
        it = util.Iterator(board)

        # 핀 번호 배열 정의
        PWM_PINS = [2, 3, 4, 5, 6, 7, 8, 9]
        DIR_PINS = [30, 31, 32, 33, 34, 35, 36, 37]
        BRK_PINS = [38, 39, 40, 41, 42, 43, 44, 45]

        # 핀 객체 리스트
        pwm_list = [board.get_pin(f'd:{pin}:p') for pin in PWM_PINS]
        dir_list = [board.get_pin(f'd:{pin}:o') for pin in DIR_PINS]
        brk_list = [board.get_pin(f'd:{pin}:o') for pin in BRK_PINS]

        # 방향 설정
        dir_list[0].write(1) # 3사분면
        dir_list[1].write(1) # 4사분면
        dir_list[2].write(0) # 1사분면
        dir_list[3].write(0) # 2사분면
        dir_list[4].write(0) # +-x축
        dir_list[5].write(1) # -y축
        dir_list[6].write(0) # +y축
        dir_list[7].write(1) # center

        for i in range(8):
            brk_list[i].write(1)  # 브레이크 ON

        return board, it, pwm_list, dir_list, brk_list


    def ReadyVision(self):

        # Camera Parameter
        K1 = np.load('Cam_Data/cam_K_1.npy')
        K2 = np.load('Cam_Data/cam_K_2.npy')
        D1 = np.load('Cam_Data/cam_D_1.npy')
        D2 = np.load('Cam_Data/cam_D_2.npy')

        # Camera 1 to 2 Transformation Matrix
        R = np.array([[0, 0, 1],
                      [0, 1, 0],
                      [-1, 0, 0]], dtype=np.float64)

        T = np.array([[-200/1000],
                      [0],
                      [200/1000]], dtype=np.float64)  # 이동 벡터 (3x1)

        # Projection Matrix
        P1 = K1 @ np.hstack((np.eye(3), np.zeros((3, 1))))
        P2 = K2 @ np.hstack((R, T.reshape(3, 1)))

        # Open Camera
        Cam1 = cv2.VideoCapture(1)
        Cam1.set(cv2.CAP_PROP_FPS, 100)
        Cam2 = cv2.VideoCapture(2)
        Cam2.set(cv2.CAP_PROP_FPS, 100)

        # Set ROI(x,y,w,h)
        ROI_1 = [50, 125, 550, 300]
        ROI_2 = [80, 60, 450, 350]

        return Cam1, Cam2, P1, P2, K1, K2, D1, D2, ROI_1, ROI_2


    def Get_Center(self, frame):

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array([40, 50, 50])
        upper = np.array([80, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts:
            c = max(cnts, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return np.array([cx, cy], dtype=np.float64)
        return None


    def Tracking(self):

        Position = []

        ret1, frame1 = self.Cam1.read()
        ret2, frame2 = self.Cam2.read()

        if not ret1 or not ret2:
            pass

        # Undistorting
        frame1_undist = cv2.undistort(frame1, self.K1, self.D1)
        frame2_undist = cv2.undistort(frame2, self.K2, self.D2)

        # Cutting ROI
        roi_frame1 = frame1_undist[self.ROI_1[1]:self.ROI_1[1] + self.ROI_1[3],
                     self.ROI_1[0]:self.ROI_1[0] + self.ROI_1[2]]
        roi_frame2 = frame2_undist[self.ROI_2[1]:self.ROI_2[1] + self.ROI_2[3],
                     self.ROI_2[0]:self.ROI_2[0] + self.ROI_2[2]]

        # Get Center
        pt1 = self.Get_Center(roi_frame1)
        pt2 = self.Get_Center(roi_frame2)

        if pt1 is not None and pt2 is not None:
            # Original Frame tuple
            pt1_orig = np.array((pt1[0] + self.ROI_1[0], pt1[1] + self.ROI_1[1]), dtype=np.float64)
            pt2_orig = np.array((pt2[0] + self.ROI_2[0], pt2[1] + self.ROI_2[1]), dtype=np.float64)

            # Triangulation
            pts4d = cv2.triangulatePoints(self.P1, self.P2, pt1_orig.T, pt2_orig.T)
            pts3d = (pts4d / pts4d[3])[:3].flatten()
            x_Cam1, y_Cam1, z_Cam1 = pts3d.flatten()

            P_Cam1 = np.array([[x_Cam1], [y_Cam1], [z_Cam1], [1]])
            P_World = self.T_World2Cam1 @ P_Cam1
            x = float(P_World[0])
            y = float(P_World[1])
            z = float(P_World[2])
            Position.append((x,y,z))

            # Visualization
            if pt1 is not None:
                cv2.circle(frame1_undist, tuple(pt1_orig.astype(int)), 5, (0, 255, 0), -1)
            if pt2 is not None:
                cv2.circle(frame2_undist, tuple(pt2_orig.astype(int)), 5, (0, 255, 0), -1)

            # Show XYZ
            cv2.putText(frame1_undist, f"3D: X={x*1000:.2f} Y={y*1000:.2f} Z={z*1000:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Show ROI
            cv2.rectangle(frame1_undist, (self.ROI_1[0], self.ROI_1[1]), (self.ROI_1[0] + self.ROI_1[2], self.ROI_1[1] + self.ROI_1[3]), (255, 0, 0), 2)
            cv2.rectangle(frame2_undist, (self.ROI_2[0], self.ROI_2[1]), (self.ROI_2[0] + self.ROI_2[2], self.ROI_2[1] + self.ROI_2[3]), (255, 0, 0), 2)

            # Show Image
            cv2.imshow("Camera 1", frame1_undist)
            cv2.imshow("Camera 2", frame2_undist)

            # Return
            return Position


    def Vision(self):

        # Time Variables
        Start_Time = time.time()
        self.Start_Time = Start_Time
        Prev_Time = 0

        # Initial Value
        XYZT_Data = []
        Z = 0
        Prev_Z = 0

        while self.running:

            Curr_Time = time.time()
            dT = Curr_Time - Prev_Time

            # RealTime Tracking
            Position = self.Tracking()

            if Position:
                XYZT_Data.append(Position[0] + (Curr_Time - Start_Time,))
                Z = float(Position[0][2])
                Prev_Time = Curr_Time
            else:
                Z = Prev_Z

            Prev_Z = Z

            # Threading Lock
            with self.lock:
                self.Z = Z
                self.dT = dT

            if cv2.waitKey(1) == 27:  # ESC
                self.running = False
                break

        self.Cam1.release()
        self.Cam2.release()
        cv2.destroyAllWindows()

        with open('Results_Data/XYZT_Data.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'z', 'time'])
            for row in XYZT_Data:
                writer.writerow(row)


    def Control(self):

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