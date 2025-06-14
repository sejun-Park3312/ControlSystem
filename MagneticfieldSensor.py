import numpy as np
import time
from scipy.optimize import minimize
from numpy import pi
from gdx import gdx

class MagneticFieldSensor:
    def __init__(self, connection='ble'):
        self.gdx = gdx.gdx()
        self.connection = connection
        self.sensors = []

    def open_connection(self):
        self.gdx.open(connection=self.connection)
        print(f"Connected to GDX with {self.connection}.")

    def select_sensors(self, sensor_ids):
        self.sensors = sensor_ids
        self.gdx.select_sensors(self.sensors)
        print(f"Selected sensors: {sensor_ids}")

    def start_measurement(self, interval=10):
        self.gdx.start(interval)
        print(f"Measurement started with {interval}ms interval.")

    def get_enabled_sensor_info(self):
        column_headers = self.gdx.enabled_sensor_info()
        print('\nEnabled Sensors:')
        print(column_headers)

    def read_magnetic_field(self):
        readings = []
        for _ in range(50):
            measurements = self.gdx.read()
            if measurements is not None:
                B_vec = np.array(measurements[:3]).reshape(3, 1)
                readings.append(B_vec)
            time.sleep(0.01)

        if len(readings) == 0:
            return None

        avg_field = np.mean(readings, axis=0)
        
        return avg_field

    def stop_and_close(self):
        self.gdx.stop()
        self.gdx.close()
        print("Measurement stopped and connection closed.")

class DipoleEstimator:
    def __init__(self, mu_0=4 * pi * 1e-7):
        self.mu = mu_0
        self.B_measured = None
        self.m = None

    def set_dipole_moment(self, m_vec):
        self.m = np.array(m_vec).reshape(3, 1)

    def set_measured_B(self, B_mT):
        self.B_measured = np.array(B_mT).reshape(3, 1) * 1e-3  # mT → T

    def dipole_field(self, r_vec):
        r = np.array(r_vec).reshape(3, 1)
        norm_r = np.linalg.norm(r)
        if norm_r == 0:
            return np.full((3, 1), np.nan)
        I = np.eye(3)
        rrT = r @ r.T
        B = (self.mu / (4 * pi)) * ((3 * rrT) / norm_r**5 - I / norm_r**3) @ self.m
        return B

    def estimate_position(self, initial_guess=None):
        if self.B_measured is None or self.m is None:
            raise ValueError("B_measured와 m이 모두 설정되어야 합니다.")

        if initial_guess is None:
            initial_guess = np.array([0.02, 0, 0])

        def loss(r_vec):
            B_model = self.dipole_field(r_vec)
            diff = self.B_measured - B_model
            return np.sum(diff ** 2)

        bounds = [(-0.05, 0.05), (-0.05, 0.05), (-0.05, 0.05)]
        result = minimize(loss, initial_guess, method='L-BFGS-B', bounds=bounds)
        
        return result.x * 1e3  # m → mm
