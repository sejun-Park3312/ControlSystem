import numpy as np
import random
import time
from simple_pid import PID
from MakeFuns import MakeFuns

MF = MakeFuns()
Ts = 20/1000
setpoint = MF.Ref_z
pid = PID(Kp = 1e-1, Kd = 1e-2, Ki = 0, setpoint = setpoint)
pid.sample_time = Ts
Buoyancy = 0.005146777750500
I_max = 3

# 실시간 루프 시작
while True:
    # 실제 시스템에서는 여기서 센서값을 받아옵니다
    z_target = random.uniform(-10/1000, 10/1000)  # 예시용 측정값 (센서 데이터 대체)

    # PID 제어 계산
    F_need = pid(z_target)
    I_input = (F_need - MF.MagnetArray_Force(z_target) - Buoyancy)/MF.CoilArray_ACoeff(z_target)
    I_input = np.round(np.clip(I_input,-I_max,I_max)/0.02)*0.02

    # 제어 입력 출력 (예: 모터 속도, PWM 값 등)
    print(f"측정값: {z_target:.2f}, 제어입력: {I_input.item():.2f}")

    # 제어 주기만큼 기다림
    time.sleep(pid.sample_time)


