import numpy as np
from MakeFuns import MakeFuns
from BasicMagnetFuns import BasicMagnetFuns

MF = MakeFuns()
BF = BasicMagnetFuns()

r_source2target = np.transpose(np.array([0, 0, 1]))
m_target = np.transpose(np.array([0, 0, 1]))
m_source = np.transpose(np.array([0, 1, 0]))

B = BF.Cal_MagnetField(r_source2target, m_source)
F = BF.Cal_MagnetForce(r_source2target, m_source, m_target)
T = BF.Cal_MagnetTorque(r_source2target, m_source, m_target)

z_target = 10/1000
F_MA = MF.MagnetArray_Force(z_target)
A_Coeff = MF.CoilArray_ACoeff(z_target)

print(F_MA[2])
print(A_Coeff)


