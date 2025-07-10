import serial
import time
import keyboard  # ESC 키 감지용

# 1️⃣ 시리얼 연결
ser = serial.Serial('COM4', 115200)
time.sleep(2)  # 아두이노 리셋 대기

print("start!")
pwm = 120
try:
    while True:
        # ESC 누르면 종료
        if keyboard.is_pressed('esc'):
            ser.write(f"{0}\n".encode())
            ser.write(b"999\n")
            print("ESC 눌림 → STOP 명령어 전송 (999)")
            break

        ser.write(f"{pwm}\n".encode())
        time.sleep(10/1000)

except KeyboardInterrupt:
    print("Ctrl+C 종료됨.")

finally:
    ser.close()
    print("Serial closed.")
