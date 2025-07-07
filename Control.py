import numpy as np
import math
import time
import threading
from simple_pid import PID
from BasicMagnetFuns import BasicMagnetFuns

class Control:
    def __init__(self):

        # BasicMagnetFuns
        self.BF = BasicMagnetFuns()

        # Coil/Magent Array
        Data = np.load("Array_Data/Data.npz")
        self.C_Points = Data['C_Points']
        self.C_Angles = Data['C_Angles']
        self.M_Points = Data['M_Points']
        self.M_Angles = Data['M_Angles']

        # DipoleMoment Magnitude
        self.Ms = 2
        self.Mc = 0.92 # NA
        self.Mt = 0.0265

        # Reference Distance
        self.Ref_z = 90/1000

        # Control Setting
        self.z_offset = 0/1000
        self.SamplingTime = 20 / 1000
        self.F_Buoyance = 0.005146777750500
        self.I_Max = 1.5
        self.Ref_Movement = 10/1000 # Target과 Array간 거리를 이만큼 줄이겠다
        self.PID_Gain =1e-1
        self.pid = PID(Kp=self.PID_Gain, Kd=self.PID_Gain/10, Ki=0, setpoint = 0)
        self.pid.sample_time = self.SamplingTime

        # Else
        self.Running = True
        self.I = 0

        self.lock = threading.Lock()

    def Angle2Direction(self, Angle):
        Direction = np.array([math.sin(Angle[1])*math.cos(Angle[0]), math.sin(Angle[1])*math.sin(Angle[0]), math.cos(Angle[1])])
        return Direction


    def MagnetArray_Force(self, z_target):

        M_Points = self.M_Points
        M_Angles = self.M_Angles
        m_target = np.array([1, 0, 0]) * self.Mt

        F = np.array([[0],[0],[0]])
        for i in range(M_Points.shape[0]):
            m_source = self.Angle2Direction(M_Angles[i,:]) * self.Ms
            r_source2target = np.array([[0,0,z_target]]) - M_Points[i,:]
            F = F + self.BF.Cal_MagnetForce(r_source2target, m_source, m_target)

        Fz = F[2]
        return Fz


    def CoilArray_ACoeff(self, z_target):

        C_Points = self.C_Points
        C_Angles = self.C_Angles
        m_target = np.array([1, 0, 0]) * self.Mt

        A_vec = np.array([[0], [0], [0]])
        for i in range(C_Points.shape[0]):
            m_source_i = self.Angle2Direction(C_Angles[i, :]) * self.Mc
            r_source2target = np.array([[0, 0, z_target]]) - C_Points[i, :]
            A_vec = A_vec + self.BF.Cal_MagnetForce(r_source2target, m_source_i, m_target)

        Az_Coeff = A_vec[2]
        return Az_Coeff


    def Get_Current(self, z):

        while self.Running:
            time.sleep(self.SamplingTime)
            with self.lock:
                F_need = self.pid(z)
                I_input = (F_need - self.MagnetArray_Force(z + self.Ref_Movement) - self.F_Buoyance) / self.CoilArray_ACoeff(z + self.Ref_Movement)
                self.I = float(np.round(np.clip(I_input, 0, self.I_Max) / 0.02) * 0.02)
