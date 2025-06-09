import numpy as np
import math


class BasicMagnetFuns:

    def __init__(self):
        self.myu_0 = 4 * math.pi * 1e-7


    def col2row(self, vec):

        vec = np.asarray(vec)
        if vec.ndim == 1:
            return vec.reshape(1, -1)  # 1차원 → (1,n)
        elif vec.shape[1] == 1:
            return vec.T  # (n,1) → (1,n)
        elif vec.shape[0] == 1:
            return vec  # 이미 (1,n)
        else:
            raise ValueError("입력 벡터가 (1, n) 또는 (n, 1) 형태여야 합니다.")



    def row2col(self, vec):

        vec = np.asarray(vec)
        if vec.ndim == 1:
            return vec.reshape(-1, 1)  # 1차원 → (n,1)
        elif vec.shape[0] == 1:
            return vec.T  # (1,n) → (n,1)
        elif vec.shape[1] == 1:
            return vec  # 이미 (n,1)
        else:
            raise ValueError("입력 벡터가 (1, n) 또는 (n, 1) 형태여야 합니다.")


    def Cal_MagnetField(self, r_source2target, m_source):

        # row vector 2 column vector
        r_source2target = self.row2col(r_source2target)
        m_source = self.row2col(m_source)

        r_hat = r_source2target / np.linalg.norm(r_source2target)
        C = self.myu_0 / (4 * math.pi * np.linalg.norm(r_source2target) ** 3)
        B = C * (3 * r_hat @ r_hat.T - np.eye(3)) @ m_source

        return B


    def Cal_MagnetTorque(self, r_source2target, m_source, m_target):

        # row vector 2 column vector
        m_target = self.row2col(m_target)

        B = self.Cal_MagnetField(r_source2target, m_source)
        T = np.cross(m_target.reshape(-1), B.reshape(-1))
        T = T.reshape(-1,1)

        return T

    def Cal_MagnetForce(self, r_source2target, m_source, m_target):

        # row vector 2 column vector
        r_source2target = self.row2col(r_source2target)
        m_source = self.row2col(m_source)
        m_target = self.row2col(m_target)

        r_hat = r_source2target / np.linalg.norm(r_source2target)
        C = 3*self.myu_0 / (4 * math.pi * np.linalg.norm(r_source2target) ** 4)
        F = C * ((r_hat.T @ m_target) * m_source + (r_hat.T @ m_source) * m_target
             + (m_source.T @ m_target - 5*(r_hat.T @ m_source)*(r_hat.T @ m_target)) * r_hat)

        return F