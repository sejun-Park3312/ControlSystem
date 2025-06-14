from oopclass import ArduinoController
from MagneticfieldSensor import MagneticFieldSensor, DipoleEstimator
from DipoleForceController import DipoleForceController
import numpy as np
import threading
import time

def main():
    port = 'COM3'
    controller = ArduinoController(port)

    sensor = MagneticFieldSensor()
    sensor.open_connection()
    sensor.select_sensors([4, 5, 6])
    sensor.start_measurement(10)
    sensor.get_enabled_sensor_info()

    estimator = DipoleEstimator()
    estimator.set_dipole_moment([0.83, 0.01, 0.01])

    coil_positions = [
        np.array([-0.045, 0.0, 0.0]),
        np.array([ 0.000, 0.045, 0.0]),
        np.array([ 0.045, 0.0, 0.0]),
        np.array([ 0.000, -0.045, 0.0])
    ]
    controller_force = DipoleForceController(estimator.m, coil_positions)

    shared = {
        "done": False,
        "r_current": np.zeros(3),
        "r_estimate_mm": np.zeros(3),
        "last_update": time.time()
    }

    def measure_loop():
        while not shared["done"]:
            B_mT = sensor.read_magnetic_field()
            if B_mT is not None:
                estimator.set_measured_B(B_mT)
                r_mm = estimator.estimate_position()
                shared["r_estimate_mm"] = r_mm
                shared["r_current"] = np.array([r_mm[2], r_mm[1], 0.0]) * 1e-3
                shared["last_update"] = time.time()
            time.sleep(0.1)

    t = threading.Thread(target=measure_loop)
    t.start()

    try:
        while True:
            user_input = input("🎯 목표 위치 입력 (x y)[mm], 종료:q: ").strip()
            if user_input.lower() == 'q':
                shared["done"] = True
                break

            try:
                x_t, y_t = map(float, user_input.split())
                r_target = np.array([x_t, y_t, 0.0]) * 1e-3

                while True:
                    r_current = shared["r_current"]
                    error = r_target - r_current
                    err_norm = np.linalg.norm(error)

                    # 가장 최신 위치 정보로 출력
                    latest_mm = shared["r_estimate_mm"]
                    print(f"📍 현재 위치: {np.round(latest_mm, 2)} mm, 오차: {err_norm * 1000:.2f} mm")

                    if err_norm < 3e-3:
                        print("✅ 목표 위치 도달")
                        controller.all_off()
                        break

                    I_vec = controller_force.compute_currents(r_current, r_target, K=20)
                    print(f"⚡ 전류 벡터 [A]: {np.round(I_vec, 3)}")
                    I_vec = np.sign(I_vec) * np.sqrt(np.abs(I_vec))
                    
                    controller.apply_currents(I_vec)
                    time.sleep(0.1)
                    
                    controller.all_off()
                    time.sleep(0.05)

            except Exception as e:
                print("❌ 입력 오류:", e)

    except KeyboardInterrupt:
        shared["done"] = True
    finally:
        controller.all_off()
        controller.close()
        sensor.stop_and_close()
        t.join()
        print("🛑 프로그램 종료")

if __name__ == "__main__":
    main()
