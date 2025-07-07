from Vision import Vision
from Control import Control
import threading
import time

CT = Control()
VS = Vision()

# Vision Thread
vision_thread = threading.Thread(target=VS.Tracking)
vision_thread.daemon = True
vision_thread.start()

# Control Thread
control_thread = threading.Thread(target=CT.Get_Current, args = (VS.Z,))
control_thread.daemon = True
control_thread.start()

# Start(Esc 누르면 탈출)
Running = True
try:
    while VS.Running:
        time.sleep(1)
        print(f"Z: {VS.Z * 1000:.2f} mm, A: {CT.I:.2f} A")

except KeyboardInterrupt:
    Running = False  # 종료 플래그 내려서 두 스레드 종료
    vision_thread.join()
    control_thread.join()