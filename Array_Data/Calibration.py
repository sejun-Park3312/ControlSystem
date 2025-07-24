import cv2
import numpy as np

CHECKERBOARD = (6, 4)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# 체커보드의 실제 3D 좌표 생성 (Z=0 평면)
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

objpoints = []  # 실제 좌표 (3D)
imgpoints = []  # 이미지 상의 좌표 (2D)

Cam1 = cv2.VideoCapture('/dev/video4', cv2.CAP_V4L2)  # USB 캠 1
Cam1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
Cam1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
sample_count = 0

print("▶ 스페이스바로 체커보드 샘플 저장")
print("▶ ESC 누르면 수집된 데이터로 캘리브레이션 수행")

while True:
    ret, frame = Cam1.read()
    if not ret:
        break

    display = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(
        gray, CHECKERBOARD,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    if found:
        cv2.drawChessboardCorners(display, CHECKERBOARD, corners, found)
        cv2.putText(display, "good", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    else:
        cv2.putText(display, "bad", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    cv2.putText(display, f" sample num: {sample_count}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    cv2.imshow("Webcam Calibration", display)
    key = cv2.waitKey(1)

    if key == 27:  # ESC
        break
    elif key == 32 and found:  # Spacebar
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        objpoints.append(objp)
        sample_count += 1
        print(f"샘플 저장됨: {sample_count}")

Cam1.release()
cv2.destroyAllWindows()

if sample_count >= 3:  # 최소 3장 이상 있어야 캘리브레이션이 의미 있음
    print("\n📐 캘리브레이션 수행 중...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    np.save("../Cam_Data/cam_K2.npy", mtx)
    np.save("../Cam_Data/cam_D2.npy", dist)
    print("카메라 행렬:\n", mtx)
    print("왜곡 계수:\n", dist)
else:
    print("❌ 캘리브레이션 실패: 최소 3개 이상의 샘플이 필요합니다.")
