from simple_pid import PID
from BasicMagnetFuns import BasicMagnetFuns
import time
import math
import threading
import numpy as np

class Control:
    def __init__(self):
        print("Controller Initializing...")
        # Basic Magnet Functions
        self.BF = BasicMagnetFuns()

        # Distance Offsets
        self.Z_Reference = 100 / 1000 # system(센터 코일 높이)과 Target 사이 Reference 거리
        self.Z_System = 100/1000 # system 높이(World 좌표계 기준)
        self.Z_Target = 0 # Target 높이(World 좌표계 기준)

        # Array
        self.C_Points, self.C_Angles, self.M_Points, self.M_Angles = self.Array()

        # DipoleMoment Magnitude
        self.Ms = 2
        self.Mc = 0.92 # NA
        self.Mt = 0.0265

        # Mechanical Properties
        self.F_Buoyance = 0.005146777750500
        self.Weight = 0.006776951342543
        self.I_Max = 1.5

        # PID
        self.SamplingTime = 25 / 1000
        self.PID_Gain =1e-1
        self.pid = PID(Kp=self.PID_Gain, Kd=self.PID_Gain/10, Ki=0, setpoint = 0)
        self.pid.sample_time = self.SamplingTime

        print("Controller Ready!")


    def Array(self):
        # Coil/Magent Array
        Data = np.load("Array_Data/Data.npz")
        C_Points = Data['C_Points']
        C_Angles = Data['C_Angles']
        M_Points = Data['M_Points']
        M_Angles = Data['M_Angles']

        C_Points[:,2] = 0
        M_Points[:,2] = 0
        return C_Points, C_Angles, M_Points, M_Angles


    def Angle2Direction(self, Angle):
        Direction = np.array([math.sin(Angle[1]) * math.cos(Angle[0]), math.sin(Angle[1]) * math.sin(Angle[0]), math.cos(Angle[1])])
        return Direction


    def MagnetArray_Force(self):
        m_target = np.array([1, 0, 0]) * self.Mt
        F = np.array([[0], [0], [0]])
        for i in range(self.M_Points.shape[0]):
            m_source = self.Angle2Direction(self.M_Angles[i, :]) * self.Ms
            # World 좌표계 기준
            r_source2target = np.array([[0, 0, self.Z_Target]]) - (self.M_Points[i, :] + np.array([0, 0, self.Z_System + 40/1000]))
            F = F + self.BF.Cal_MagnetForce(r_source2target, m_source, m_target)

        Fz = F[2]
        return Fz


    def CoilArray_ACoeff(self):
        m_target = np.array([1, 0, 0]) * self.Mt
        A_vec = np.array([[0], [0], [0]])
        for i in range(self.C_Points.shape[0]):
            m_source_i = self.Angle2Direction(self.C_Angles[i, :]) * self.Mc
            # World 좌표계 기준
            r_source2target = np.array([[0, 0, self.Z_Target]]) - (self.C_Points[i, :] + np.array([0, 0, self.Z_System]))
            A_vec = A_vec + self.BF.Cal_MagnetForce(r_source2target, m_source_i, m_target)

        Az_Coeff = A_vec[2]
        return Az_Coeff


    def Get_PWM(self):
        Z_Error = self.Z_Reference - (self.Z_System - self.Z_Target)
        F_pid = self.pid(Z_Error, dt = self.SamplingTime)
        I = (F_pid - self.MagnetArray_Force() - self.F_Buoyance + self.Weight) / self.CoilArray_ACoeff()
        PWM = round(float(np.clip(I, 0, self.I_Max) * 255 / self.I_Max))
        return PWM
