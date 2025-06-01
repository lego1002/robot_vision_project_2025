import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os
import time
from typing import Dict

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hough_config.json")

class HoughHoleDetector(Node):
    def __init__(self):
        super().__init__('hough_hole_detector')
        self.bridge = CvBridge()

        # 讀取 Hough 參數設定
        with open(CONFIG_PATH, 'r') as f:
            self.hough_config = json.load(f)

        self.views: Dict[str, np.ndarray] = {}
        self.last_update_time: Dict[str, float] = {}

        self.good_sub = self.create_subscription(Image, '/edges/good_parts', self.callback_good, 10)
        self.bad_sub = self.create_subscription(Image, '/edges/bad_parts', self.callback_bad, 10)

        self.count_pub = self.create_publisher(Int32, '/hole_detector/hole_count', 10)
        self.center_pub = self.create_publisher(String, '/hole_detector/hole_centers', 10)

    def process(self, msg):
        try:
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            frame_info = msg.header.frame_id
            part_id = frame_info.split(":")[0]
            status = frame_info.split(":")[1] if ":" in frame_info else "unknown"
            key = f"{part_id}:{status}"

            cfg = self.hough_config
            circles = cv2.HoughCircles(
                gray,
                cv2.HOUGH_GRADIENT,
                dp=cfg.get("dp", 0.12),
                minDist=cfg.get("minDist", 20),
                param1=cfg.get("param1", 100),
                param2=cfg.get("param2", 22),
                minRadius=cfg.get("minRadius", 8),
                maxRadius=cfg.get("maxRadius", 15)
            )

            hole_count = 0
            hole_centers = []

            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i, (x, y, r) in enumerate(circles[0, :]):
                    cv2.circle(color, (x, y), r, (0, 255, 0), 2)
                    cv2.putText(color, str(i + 1), (x - 5, y - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    hole_centers.append((int(x), int(y)))
                    hole_count += 1

            # 發布圓心資料
            center_msg = String()
            center_data = {
                "id": part_id,
                "centers": hole_centers
            }
            center_msg.data = json.dumps(center_data)  # 轉成 JSON 字串
            self.center_pub.publish(center_msg)

            # 顯示畫面
            resized = cv2.resize(color, (500, 150))
            label = f"{part_id} [{status}]"
            cv2.putText(resized, label, (5, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0) if status == "good" else (0, 0, 255), 2)

            self.views[key] = resized
            self.last_update_time[key] = time.time()

            # 清除過期畫面
            now = time.time()
            expired_keys = [k for k, t in self.last_update_time.items() if now - t > 0.15]
            for k in expired_keys:
                self.views.pop(k)
                self.last_update_time.pop(k)

            # 合併顯示
            sorted_items = sorted(self.views.items())
            combined = np.vstack([img for _, img in sorted_items])
            if combined is not None and combined.shape[1] > 0:
                cv2.imshow("Hough Circle Combined", combined)
                cv2.waitKey(1)

            # 發布孔洞數
            self.count_pub.publish(Int32(data=hole_count))
            self.get_logger().info(f"🔵 {frame_info} 偵測到 {hole_count} 個孔洞")

        except Exception as e:
            self.get_logger().error(f"❌ Hough 圓偵測錯誤: {e}")

    def callback_good(self, msg):
        self.process(msg)

    def callback_bad(self, msg):
        self.process(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HoughHoleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 手動關閉 HoughHoleDetector")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()