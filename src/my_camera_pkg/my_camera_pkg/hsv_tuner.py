import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os

CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "hsv_config.json"
)

def nothing(x):
    pass

# 建立視窗與滑桿
cv2.namedWindow("HSV Tuner", cv2.WINDOW_NORMAL)
cv2.resizeWindow("HSV Tuner", 1440, 960)

cv2.createTrackbar("Min H", "HSV Tuner", 0, 179, nothing)
cv2.createTrackbar("Max H", "HSV Tuner", 179, 179, nothing)
cv2.createTrackbar("Min S", "HSV Tuner", 0, 255, nothing)
cv2.createTrackbar("Max S", "HSV Tuner", 255, 255, nothing)
cv2.createTrackbar("Min V", "HSV Tuner", 0, 255, nothing)
cv2.createTrackbar("Max V", "HSV Tuner", 255, 255, nothing)

class HSVTunerNode(Node):
    def __init__(self):
        super().__init__('hsv_tuner_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_rect',
            self.image_callback,
            10
        )
        self.get_logger().info("🎨 HSV Tuner ROS node started. 按 's' 儲存設定，'q' 離開。")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            minH = cv2.getTrackbarPos("Min H", "HSV Tuner")
            maxH = cv2.getTrackbarPos("Max H", "HSV Tuner")
            minS = cv2.getTrackbarPos("Min S", "HSV Tuner")
            maxS = cv2.getTrackbarPos("Max S", "HSV Tuner")
            minV = cv2.getTrackbarPos("Min V", "HSV Tuner")
            maxV = cv2.getTrackbarPos("Max V", "HSV Tuner")

            lower = np.array([minH, minS, minV])
            upper = np.array([maxH, maxS, maxV])
            mask = cv2.inRange(hsv, lower, upper)
            result = cv2.bitwise_and(frame, frame, mask=mask)

            cv2.imshow("HSV Tuner", result)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                params = {
                    "minH": minH, "maxH": maxH,
                    "minS": minS, "maxS": maxS,
                    "minV": minV, "maxV": maxV
                }
                with open(CONFIG_PATH, "w") as f:
                    json.dump(params, f, indent=4)
                self.get_logger().info(f"✅ 已儲存 HSV 參數到 {CONFIG_PATH}")

            elif key == ord('q'):
                self.get_logger().info("🛑 手動結束 HSV Tuner")
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"❌ 圖像處理錯誤: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HSVTunerNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
