import numpy as np
import math
from BasicMagnetFuns import BasicMagnetFuns

class MakeFuns:
    def __init__(self):

        # BasicMagnetFuns
        self.BF = BasicMagnetFuns()

        # Coil/Magent Array
        Data = np.load("Data.npz")
        self.C_Points = Data['C_Points']
        self.C_Angles = Data['C_Angles']
        self.M_Points = Data['M_Points']
        self.M_Angles = Data['M_Angles']

        # DipoleMoment Magnitude
        self.Ms = 2
        self.Mc = 0.8 # NA
        self.Mt = 0.0265

        # Reference Distance
        self.Ref_z = 90/1000


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
