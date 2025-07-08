from VisionControl import VisionControl
import time
import cv2

VC = VisionControl()
VC.it.start()
time.sleep(1)
for i in range(8):
    VC.brk_list[i].write(0)  # 브레이크 Off
    VC.pwm_list[i].write(1.0)

print(f"⏹️ 드라이버 {i} 시작!!!\n")

curr_time = time.time()
# 초기 핀 번호 설정
while time.time() -curr_time > 10:

    if cv2.waitKey(1) == 27:  # ESC

        for i in range(8):
            VC.pwm_list[i].write(0)
            VC.brk_list[i].write(1)  # 브레이크 ON
            print(f"⏹️ 드라이버 {i} Off!!!\n")

        break

cv2.destroyAllWindows()
VC.board.exit()
