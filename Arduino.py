from pyfirmata import ArduinoMega, util
import time

# 보드 포트 설정
board = ArduinoMega('COM3')  # 실제 연결된 포트로 수정 필요
it = util.Iterator(board)
it.start()
time.sleep(1)  # 초기화 대기

# 핀 번호 배열 정의
PWM_PINS = [2, 3, 4, 5, 6, 7, 8, 9]
DIR_PINS = [30, 31, 32, 33, 34, 35, 36, 37]
BRK_PINS = [38, 39, 40, 41, 42, 43, 44, 45]

# 핀 객체 리스트
pwm_list = [board.get_pin(f'd:{pin}:p') for pin in PWM_PINS]
dir_list = [board.get_pin(f'd:{pin}:o') for pin in DIR_PINS]
brk_list = [board.get_pin(f'd:{pin}:o') for pin in BRK_PINS]

# 전류 테스트 루프
for i in range(8):
    print(f"▶ 드라이버 {i} 테스트 중...")

    dir_list[i].write(1)  # 방향 설정
    brk_list[i].write(0)  # 브레이크 해제
    pwm_list[i].write(1.0)  # 전류 인가 (100%)

    time.sleep(2)  # 2초간 전류 흐르게 유지

    pwm_list[i].write(0)  # PWM OFF
    brk_list[i].write(1)  # 브레이크 ON

    print(f"⏹️ 드라이버 {i} 정지됨\n")
    time.sleep(1)  # 다음 드라이버 전환 전 잠깐 대기

# 종료 처리
board.exit()
