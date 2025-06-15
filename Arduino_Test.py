from VisionControl import VisionControl
import time

VC = VisionControl()
VC.it.start()
time.sleep(1)
print(f"⏹️ 드라이버 {i} 시작!!!\n")
import cv2
import numpy as np
import time
from pyfirmata2 import Arduino, util

# 초기 핀 번호 설정
current_pin_num = 0
reverse_pin_num = 8  # 정방향 핀 + 1
current_pin = VC.board.get_pin(f'd:{current_pin_num}:o')
reverse_pin = VC.board.get_pin(f'd:{reverse_pin_num}:o')

# OpenCV 창 설정
cv2.namedWindow("Arduino Control")
font = cv2.FONT_HERSHEY_SIMPLEX

while True:
    # 상태창 이미지
    img = np.ones((200, 500, 3), dtype=np.uint8) * 255
    text1 = f"Pin Number : {current_pin_num}"
    cv2.putText(img, text1, (10, 80), font, 0.7, (0, 0, 0), 2)
    cv2.putText(img, "Press 0-7 to select pin, o/z: 2s pulse,, ESC to exit",
                (10, 170), font, 0.45, (100, 100, 100), 1)
    cv2.imshow("Arduino Control", img)

    key = cv2.waitKey(100) & 0xFF

    # 숫자 키 0~7 → 핀 변경
    if ord('0') <= key <= ord('7'):
        current_pin_num = key - ord('0')
        current_pin = VC.board.get_pin(f'd:{current_pin_num}:o')
        current_pin.write(0)
        print(f"🔁 핀 변경됨 → Forward: {current_pin_num}")

    # 'o/z' 키 → 2초 동안 one/zero dir 출력
    elif key == ord('h'):
        print(f"⚡ 핀 {current_pin_num} → ON (2초)")
        current_pin.write(1)
        time.sleep(2)
        current_pin.write(0)
        print(f"⚫ 핀 {current_pin_num} → OFF")

    # 'h' 키 → 정방향 ON
    elif key == ord('h'):
        print(f"➡️  Forward (핀 {current_pin_num}) → HIGH")
        reverse_pin.write(0)
        current_pin.write(1)

    # 'l' 키 → 역방향 ON
    elif key == ord('l'):
        print(f"⬅️  Reverse (핀 {reverse_pin_num}) → HIGH")
        current_pin.write(0)
        reverse_pin.write(1)

    # ESC → 종료
    elif key == 27:
        print("🚪 ESC 눌림 - 종료합니다.")
        break

cv2.destroyAllWindows()
board.exit()
