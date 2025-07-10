import serial
import time
import keyboard

class Arduino:
    def __init__(self):
        print("Arduino Connecting...")
        self.Running = True
        self.ArduinoSerial = serial.Serial('COM4', 115200)
        time.sleep(2)
        print("Arduino Connected!")


    def Send_PWM(self, PWM):
        if self.Running == True:
            self.ArduinoSerial.write(f"{PWM}\n".encode())
        else:
            self.Disconnect()


    def Disconnect(self):
        if self.Running == True:
            self.Running = False
            self.ArduinoSerial.write(f"{0}\n".encode())
            self.ArduinoSerial.write(b"999\n")
            self.ArduinoSerial.flush()
            self.ArduinoSerial.close()
            print("Arduino Disconnected!")


    def ManualPWM(self):
        PWM = 200
        OnOff = False
        try:
            while True:
                # ESC 누르면 종료
                if keyboard.is_pressed('esc'):
                    self.Disconnect()
                    break
                if keyboard.is_pressed('space'):
                    if not OnOff:
                        OnOff = True
                else:
                    OnOff = False

                if OnOff == True:
                    self.Send_PWM(PWM)
                else:
                    self.Send_PWM(0)

                time.sleep(50/1000)

        except KeyboardInterrupt:
            self.Disconnect()
