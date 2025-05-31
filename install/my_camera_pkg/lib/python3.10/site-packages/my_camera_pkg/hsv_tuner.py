import cv2
import numpy as np
import json
import os

# 儲存檔案路徑
CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "hsv_config.json"
)

print(os.path.dirname(os.path.abspath(__file__)))

def nothing(x):
    pass

# 建立滑桿介面
cv2.namedWindow("HSV Filter")
cv2.createTrackbar("Min H", "HSV Filter", 0, 179, nothing)
cv2.createTrackbar("Max H", "HSV Filter", 179, 179, nothing)
cv2.createTrackbar("Min S", "HSV Filter", 0, 255, nothing)
cv2.createTrackbar("Max S", "HSV Filter", 255, 255, nothing)
cv2.createTrackbar("Min V", "HSV Filter", 0, 255, nothing)
cv2.createTrackbar("Max V", "HSV Filter", 255, 255, nothing)

# 嘗試開啟攝影機
cap = cv2.VideoCapture('/dev/video0')
if not cap.isOpened():
    print("❌ 無法開啟攝影機")
    exit()

print("✅ 按 's' 鍵儲存參數，按 'q' 離開")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # BGR → HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 取得滑桿值
    minH = cv2.getTrackbarPos("Min H", "HSV Filter")
    maxH = cv2.getTrackbarPos("Max H", "HSV Filter")
    minS = cv2.getTrackbarPos("Min S", "HSV Filter")
    maxS = cv2.getTrackbarPos("Max S", "HSV Filter")
    minV = cv2.getTrackbarPos("Min V", "HSV Filter")
    maxV = cv2.getTrackbarPos("Max V", "HSV Filter")

    # 篩選影像
    lower = np.array([minH, minS, minV])
    upper = np.array([maxH, maxS, maxV])
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # 顯示
    cv2.imshow("Filtered", result)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('s'):
        params = {
            "minH": minH, "maxH": maxH,
            "minS": minS, "maxS": maxS,
            "minV": minV, "maxV": maxV
        }
        with open(CONFIG_PATH, "w") as f:
            json.dump(params, f, indent=4)
        print(f"✅ 已儲存 HSV 參數到 {CONFIG_PATH}")

cap.release()
cv2.destroyAllWindows()