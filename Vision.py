import cv2
import threading
import numpy as np
import time
import csv

class Vision:
    def __init__(self):

        print("Vision Initializing...")

        # Vision Setting
        self.Z_Offset = 15/1000
        self.SamplingTime = 20/1000
        self.Running = True
        self.XYZT_Data = []

        # Threading
        self.lock = threading.Lock()
        self.Z = 0

        # 계산되는 파라미터들...
        self.Cam1, self.Cam2, self.P1, self.P2, self.K1, self.K2, self.D1, self.D2, self.ROI_1, self.ROI_2 = self.Ready()
        self.R_World2Cam1 = np.array([[1, 0, 0],
                      [0, 0, 1],
                      [0, -1, 0]], dtype=np.float64)
        self.P_World2Cam1 = np.array([[0],
                      [-200/1000],
                      [self.Z_Offset]], dtype=np.float64)  # 이동 벡터 (3x1)
        self.T_World2Cam1 = T = np.vstack((np.hstack((self.R_World2Cam1, self.P_World2Cam1.reshape(3,1))), [[0, 0, 0, 1]]))

        print("Vision Ready!")


    def Ready(self):
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


    def Get_Position(self):

        Position = []
        ret1, frame1 = self.Cam1.read()
        ret2, frame2 = self.Cam2.read()

        if not ret1 or not ret2:
            return None

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
            Position = [x, y, z]

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


    def Tracking(self):
        self.XYZT_Data = [[0, 0, 0, 0]]  # X,Y,Z,T
        StartTime = time.time()
        CurrTime = time.time()

        print("Start Tracking!")
        while self.Running:
            AvgPosition = self.XYZT_Data[-1][0:3]
            CurrTime = time.time()

            while time.time() - CurrTime < self.SamplingTime:

                Position = self.Get_Position()
                if cv2.waitKey(1) == 27:  # ESC
                    self.Running = False
                    print("Vision Stopped")
                    break

                if Position:
                    AvgPosition = [x / 2 + y / 2 for x, y in zip(AvgPosition, Position)]

            self.XYZT_Data.append(AvgPosition + [time.time() - StartTime])

            # Threading Lock
            with self.lock:
                    self.Z = AvgPosition[2]

        print("End Tracking!")
        self.Cam1.release()
        self.Cam2.release()
        cv2.destroyAllWindows()