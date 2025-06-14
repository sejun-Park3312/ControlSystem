import numpy as np
import time

class ActivateMagnet:
    def __init__(self, relative_positions):
        self.relative_positions = relative_positions  # shape: (4, 2)
        self.K = 0.1                                   # 제어 게인
        self.activation_time = 0.3                    # 전류 인가 시간

        # 기존 주축 방향 + 0.01 성분 → 단위 벡터로 정규화
        raw_directions = [
            [ 0.01,  0.01, 1],   # Coil 1
            [ 0.01,  0.01, 1],   # Coil 2
            [ 0.01,  0.01, 1],   # Coil 3
            [ 0.01,  0.01, 1]    # Coil 4
        ]
        self.coil_directions = [np.array(v) / np.linalg.norm(v) for v in raw_directions]

    def activate_by_target(self, controller, r_target, r_current):
        # 💡 목표/현재 위치 벡터: shape (2,)
        delta = (r_target - r_current).reshape(2, 1)
        force = self.K * delta                        # shape: (2, 1)

        mu_0 = 4 * np.pi * 1e-7
        m_magnitude = 1.0
        m = m_magnitude * np.array([[0], [0], [1]])   # z방향 dipole moment

        current_vector = np.zeros((4, 1))

        for i in range(4):
            coil_pos = r_current + self.relative_positions[i]
            r_vec = np.array([coil_pos[0], coil_pos[1], 0]).reshape(3, 1)

            norm_r = np.linalg.norm(r_vec)
            r_hat = r_vec / norm_r
            I = np.eye(3)
            term = 3 * np.outer(r_hat, r_hat) - I
            dBdm = (mu_0 / (4 * np.pi * norm_r**3)) * term

            F_i = np.cross(self.coil_directions[i], (dBdm @ m), axis=0)  # shape: (3,1)
            J_i = F_i[:2]  # xy평면

            current_vector[i] = np.dot(J_i.flatten(), force.flatten())

        print(f"\n🔁 current_vector (before clip): {current_vector.flatten()}")

        for i in range(4):
            pwm_pin = controller.pwm_pins[i]
            in1_pin = controller.in1_pins[i]
            in2_pin = controller.in2_pins[i]

            # 방향 설정
            if current_vector[i] >= 0:
                controller.arduino.digital[in1_pin].write(1)
                controller.arduino.digital[in2_pin].write(0)
            else:
                controller.arduino.digital[in1_pin].write(0)
                controller.arduino.digital[in2_pin].write(1)

            # PWM 설정: 절댓값 + 최소값 보장
            duty = np.clip(abs(current_vector[i]), 0.2, 1.0)
            controller.arduino.digital[pwm_pin].write(duty)
            print(f"⚡ Coil {i+1}: PWM duty = {duty:.2f}")

        time.sleep(self.activation_time)

        for i in range(4):
            controller.arduino.digital[controller.pwm_pins[i]].write(0)
            controller.arduino.digital[controller.in1_pins[i]].write(0)
            controller.arduino.digital[controller.in2_pins[i]].write(0)
        print("⏹️ 모든 코일 비활성화 완료")
