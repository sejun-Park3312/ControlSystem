import time
from pyfirmata2 import PWM, ArduinoMega, OUTPUT

class ArduinoController:
    def __init__(self, port):
        self.pwm_pins = [11, 9, 8, 10]
        self.in1_pins = [28, 26, 24, 22]  # break
        self.in2_pins = [29, 27, 25, 23]  # dir
        self.arduino = ArduinoMega(port)

        try:
            self.arduino.samplingOn(100)
            print("✅ Arduino connected")
            self.setup_pins()
        except Exception as e:
            print(f"❌ Failed to connect to Arduino: {e}")

    def setup_pins(self):
        for pin in self.pwm_pins:
            self.arduino.digital[pin].mode = PWM

        for pin in range(22, 29):
            self.arduino.digital[pin].mode = OUTPUT
            self.arduino.digital[pin].write(0)


        # break 해제
        self.arduino.digital[self.in1_pins[0]].write(0)
        self.arduino.digital[self.in1_pins[1]].write(0)
        self.arduino.digital[self.in1_pins[2]].write(0)
        self.arduino.digital[self.in1_pins[3]].write(0)

                # 방향 고정
        self.arduino.digital[self.in2_pins[0]].write(0)
        self.arduino.digital[self.in2_pins[1]].write(0)
        self.arduino.digital[self.in2_pins[2]].write(0)
        self.arduino.digital[self.in2_pins[3]].write(1)


    def apply_currents(self, current_vector):
        """
        current_vector: 길이 4의 리스트, 각 전자석에 흘릴 전류 [A] (음수 가능)
        """
        directions = [-1, 1, 1, -1]
        for i in range(4):
            current = current_vector[i]
            direction_pin = self.in2_pins[i]
            pwm_pin = self.pwm_pins[i]
            direction = directions[i]
            # 방향 설정
            self.arduino.digital[direction_pin].write(1 if current*direction >= 0 else 0)

            # PWM 값 설정 (0.0 ~ 1.0)
            pwm_value = min(abs(current) / 1.0, 1.0)
            self.arduino.digital[pwm_pin].write(pwm_value)

    def all_off(self):
        for pin in self.pwm_pins:
            self.arduino.digital[pin].write(0)
        for pin in self.in1_pins + self.in2_pins:
            self.arduino.digital[pin].write(0)

    def close(self):
        self.all_off()
        if self.arduino:
            try:
                self.arduino.exit()
                print("🔌 Arduino 연결 종료")
            except Exception as e:
                print(f"⚠️ 종료 오류: {e}")
