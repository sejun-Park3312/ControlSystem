import cv2
import numpy as np
import time
import threading
import csv

class Vision:
    def __init__(self):
        # Vision Setting
        self.Cam1, self.Cam2, self.P1, self.P2, self.K1, self.K2, self.D1, self.D2, self.ROI_1, self.ROI_2 = self.ReadyVision()
        self.z_offset = 0 / 1000
        self.Visual = True
        self.SamplingTime = 15/1000

        # Threading
        self.lock = threading.Lock()
        self.dT = 1000
        self.Z = 0
        self.Running = True


    def ReadyVision(self):
        # Camera Parameter
        K1 = np.load('Cam_Data/cam_K_1.npy')
        K2 = np.load('Cam_Data/cam_K_2.npy')
        D1 = np.load('Cam_Data/cam_D_1.npy')
        D2 = np.load('Cam_Data/cam_D_2.npy')

        # Camera Transformation Matrix
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
        Cam1 = cv2.VideoCapture(0)
        Cam1.set(cv2.CAP_PROP_FPS, 100)
        Cam2 = cv2.VideoCapture(1)
        Cam2.set(cv2.CAP_PROP_FPS, 100)

        # Set ROI(x,y,w,h)
        ROI_1 = [80, 60, 450, 350]
        ROI_2 = [120, 125, 400, 250]

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

        # Get Center(ROI Frame tuple)
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
            x = x_Cam1
            y = z_Cam1
            z = -y_Cam1 + self.z_offset
            Position = [x,y,z]

            # Visualization
            if self.Visual == True:
                # Show Point
                cv2.circle(frame1_undist, tuple(pt1_orig.astype(int)), 5, (0, 255, 0), -1)
                cv2.circle(frame2_undist, tuple(pt2_orig.astype(int)), 5, (0, 255, 0), -1)

                # Show XYZ
                cv2.putText(frame1_undist, f"3D: X={x * 1000:.2f} Y={y * 1000:.2f} Z={z * 1000:.2f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # Show ROI
                cv2.rectangle(frame1_undist, (self.ROI_1[0], self.ROI_1[1]),
                              (self.ROI_1[0] + self.ROI_1[2], self.ROI_1[1] + self.ROI_1[3]), (255, 0, 0), 2)
                cv2.rectangle(frame2_undist, (self.ROI_2[0], self.ROI_2[1]),
                              (self.ROI_2[0] + self.ROI_2[2], self.ROI_2[1] + self.ROI_2[3]), (255, 0, 0), 2)

                # Show Image
                cv2.imshow("Camera 1", frame1_undist)
                cv2.imshow("Camera 2", frame2_undist)


            return Position
        return None


    def Tracking(self):

        Trajectory = []
        XYZT = []
        CurrPosition = self.Get_Position()

        print("Start Tracking!")
        StartTime = time.time()
        while self.Running:
            AvgPosition = CurrPosition
            CurrTime = time.time()
            while time.time() - CurrTime < self.SamplingTime:
                CurrPosition = self.Get_Position()
                if CurrPosition:
                    AvgPosition = [x/2 + y/2 for x, y in zip(AvgPosition, CurrPosition)]
            XYZT.append(time.time() - StartTime)  # 시간 추가
            Trajectory.append(AvgPosition)

            with self.lock:
                self.Z = Trajectory[-1][2]

            if self.Visual:
                if cv2.waitKey(1) == 27:  # ESC 누르면
                    self.Running = False

        self.Cam1.release()
        self.Cam2.release()
        cv2.destroyAllWindows()
        print("End Tracking!")

        with open('Results_Data/Trajectory.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'z', 'time'])
            for row in Trajectory:
                writer.writerow(row)


    def Start(self):

        # Vision Thread
        vision_thread = threading.Thread(target=self.Tracking)
        vision_thread.daemon = True
        vision_thread.start()

        # Start(Esc 누르면 탈출)
        try:
            while self.Running:
                time.sleep(1)
                print(f"Z: {self.Z*1000:.2f} mm")
                if cv2.waitKey(1) == 27:  # ESC
                    self.Running = False
                    break
        except KeyboardInterrupt:
            self.Running = False

