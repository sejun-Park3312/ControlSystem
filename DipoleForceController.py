import numpy as np
from numpy.linalg import pinv

class DipoleForceController:
    def __init__(self, dipole_moment, coil_positions, mu=4 * np.pi * 1e-7):
        self.m = np.array(dipole_moment).reshape(3, 1)
        self.coil_positions = coil_positions
        self.mu = mu

    def compute_matrix_A(self, r_mag):
        A = []
        for coil_pos in self.coil_positions:
            r_ij = r_mag - coil_pos
            r_hat = r_ij / np.linalg.norm(r_ij)
            m_new = np.array([0,0,1])
            f_i = (3 * self.mu) / (4 * np.pi * np.linalg.norm(r_ij) ** 4) * (2*np.dot(r_hat,m_new)*m_new  + (1-5*np.dot(r_hat, m_new)**2)*r_hat)
            A.append(f_i.flatten())
        return np.array(A).T

    def compute_currents(self, r_current, r_target, K):
        error = r_target - r_current
        F = K * error.reshape(3, 1)
        A = self.compute_matrix_A(r_current)
        A_ = A[0:2]
        F_ = F[0:2]
        I = np.linalg.pinv(A_) @ F_
        
        return I.flatten()

