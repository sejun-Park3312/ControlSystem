from Arduino import Arduino
import time
import keyboard  # ESC 키 감지용

AD = Arduino()
PWM = 200                                    
OnOff = False
try:
    while True:
        # ESC 누르면 종료
        if keyboard.is_pressed('esc'):
            AD.Disconnect()
            break

        if keyboard.is_pressed('space'):
            if not OnOff:
                OnOff = True  # 눌렸다고 표시
        else:
            OnOff = False  # 뗐으면 다시 대기 상태로

        if OnOff == True:
            AD.Send_PWM(PWM)
        else:
            AD.Send_PWM(0)

        time.sleep(50/1000)

except KeyboardInterrupt:
    AD.Disconnect()

finally:
    AD.Disconnect()
