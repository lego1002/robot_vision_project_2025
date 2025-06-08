import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os

class HSVFilterNode(Node):
    def __init__(self):
        super().__init__('hsv_filter_node')
        self.bridge = CvBridge()

        # === Load HSV Config ===
        CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hsv_config.json")
        try:
            with open(CONFIG_PATH, "r") as f:
                config = json.load(f)
                self.lower = np.array([config['minH'], config['minS'], config['minV']])
                self.upper = np.array([config['maxH'], config['maxS'], config['maxV']])
                self.get_logger().info(f"✅ HSV Config: {self.lower} ~ {self.upper}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load HSV config: {e}")
            self.lower = np.array([0, 0, 0])
            self.upper = np.array([179, 255, 255])

        # === Subscribers & Publishers ===
        self.subscription = self.create_subscription(Image, '/camera/image_rect', self.callback, 10)
        self.yolo_count_sub = self.create_subscription(Int32, '/yolo/part_count', self.yolo_count_callback, 10)
        self.yolo_center_sub = self.create_subscription(String, '/yolo/part_centers', self.yolo_center_callback, 10)
        self.analysis_pub = self.create_publisher(String, '/analysis/part_difference', 10)

        self.latest_yolo_count = None
        self.latest_yolo_centers = []

    def yolo_count_callback(self, msg):
        self.latest_yolo_count = msg.data

    def yolo_center_callback(self, msg):
        try:
            self.latest_yolo_centers = eval(msg.data)  # [(x, y), ...]
        except Exception:
            self.latest_yolo_centers = []

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower, self.upper)

            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))
            thick = cv2.dilate(mask, kernel, iterations=1)
            contours, _ = cv2.findContours(thick, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            result_img = cv2.bitwise_and(frame, frame, mask=mask)

            # combined = np.hstack((frame, mask_bgr, result_img))
            # combined_resized = cv2.resize(combined, (900, 300))

            resized = cv2.resize(result_img, (960, 540))
            cv2.imshow("HSV Filtering", resized)
            cv2.waitKey(1)

            hsv_boxes = []
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area >= 3000:
                    x, y, w, h = cv2.boundingRect(cnt)
                    hsv_boxes.append((x, y, w, h))

            # 依 y 座標排序：從上往下為位置 1~4
            hsv_boxes.sort(key=lambda b: b[1])
            hsv_count = len(hsv_boxes)
            yolo_count = self.latest_yolo_count

            message = ""
            expected_count = 4

            if hsv_count > expected_count:
                diff = hsv_count - expected_count
                message = f"多出 {diff} 個額外零件"

            center_count = len(self.latest_yolo_centers)

            if hsv_count == expected_count:
                if center_count < expected_count:
                    lost_positions = []
                    for i, (x, y, w, h) in enumerate(hsv_boxes):
                        found = any(
                            x <= cx <= x + w and y <= cy <= y + h
                            for (cx, cy) in self.latest_yolo_centers
                        )
                        if not found:
                            lost_positions.append(i + 1)  # 位置從 1 開始

                    if lost_positions:
                        lost_str = ", ".join(map(str, lost_positions))
                        message = f"位置 {lost_str} 的零件可能變形"
                    else:
                        message = f"有 {expected_count - center_count} 個零件變形"
                elif center_count == expected_count:
                    message = "所有零件完整 ✅"

            elif hsv_count < expected_count:
                message = f"缺少 {expected_count - hsv_count} 個零件"

            if message:
                result = String()
                result.data = f"{message}" #[HSV:{hsv_count}, YOLO:{yolo_count}] => 
                self.analysis_pub.publish(result)
                self.get_logger().info(result.data)

        except Exception as e:
            self.get_logger().error(f"❌ HSV 分析錯誤: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HSVFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 HSV 分析節點結束")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
