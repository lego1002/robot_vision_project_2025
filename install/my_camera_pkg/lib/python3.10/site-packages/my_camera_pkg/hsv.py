import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os

class HSVFilterNode(Node):
    def __init__(self):
        super().__init__('hsv_filter_node')
        self.bridge = CvBridge()

        # 讀取 HSV 設定值
        CONFIG_PATH = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "hsv_config.json"
        )

        try:
            with open(CONFIG_PATH, "r") as f:
                config = json.load(f)
                self.lower = np.array([config['minH'], config['minS'], config['minV']])
                self.upper = np.array([config['maxH'], config['maxS'], config['maxV']])
                self.get_logger().info(f"✅ 成功讀取 HSV 設定：{self.lower} ~ {self.upper}")
        except Exception as e:
            self.get_logger().error(f"❌ 讀取 HSV 設定檔失敗: {e}")
            self.lower = np.array([0, 0, 0])
            self.upper = np.array([179, 255, 255])

        # 訂閱原始影像
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10)

        # 發布篩選後影像
        self.publisher = self.create_publisher(
            Image,
            '/camera/image_hsv',
            10)

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower, self.upper)
            result = cv2.bitwise_and(frame, frame, mask=mask)

            # 顯示 HSV 畫面（轉回 BGR 讓人眼可看）
            hsv_bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            combined = np.hstack((frame, hsv_bgr, result))  # 原圖 | HSV | 篩選後

            cv2.imshow("HSV Debug View", combined)
            cv2.waitKey(1)

            result_msg = self.bridge.cv2_to_imgmsg(result, encoding='bgr8')
            result_msg.header = msg.header
            self.publisher.publish(result_msg)

            self.get_logger().info("📤 已發布 HSV 篩選結果 /camera/image_hsv")
            
        except Exception as e:
            self.get_logger().error(f"❌ HSV 篩選錯誤: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HSVFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 結束 hsv_filter_node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
