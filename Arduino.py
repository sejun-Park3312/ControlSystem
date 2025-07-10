import serial
import time

class Arduino:
    def __init__(self):

        self.Running = True
        self.ArduinoSerial = serial.Serial('COM4', 115200)
        time.sleep(2)
        print("Arduino Connected!")


    def Send_PWM(self, PWM):

        if self.Running == True:
            self.ArduinoSerial.write(f"{PWM}\n".encode())


    def Disconnect(self):

        if self.Running == True:
            self.Running = False
            self.ArduinoSerial.write(f"{0}\n".encode())
            self.ArduinoSerial.write(b"999\n")
            self.ArduinoSerial.flush()
            self.ArduinoSerial.close()
            print("Arduino Disconnected!")

