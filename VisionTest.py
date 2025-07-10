from Vision import Vision
import time
import keyboard
import threading

print("Ready Vision...")
VS = Vision()

try:
    print("Vision Started!")
    while True:
        # threading.Thread(target=VS.Tracking, daemon=True)
        VS.Tracking()
        # ESC 누르면 종료
        if keyboard.is_pressed('esc'):
            print("Closing...")
            VS.CloseVision()
            print("Vision Closed!")
            break

        print(VS.Z)
        time.sleep(3)


except KeyboardInterrupt:
    VS.CloseVision()

finally:
    VS.CloseVision()