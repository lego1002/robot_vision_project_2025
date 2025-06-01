import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import json
import cv2
import numpy as np
from cv_bridge import CvBridge
from typing import Dict
import time

class HoleTemplateMatcher(Node):
    def __init__(self):
        super().__init__('hole_template_matcher')
        self.bridge = CvBridge()

        self.center_sub = self.create_subscription(String, '/hole_detector/hole_centers', self.center_callback, 10)
        self.image_sub = self.create_subscription(Image, '/yolo/good_parts', self.image_callback, 10)

        self.problem_pub = self.create_publisher(String, '/hole_template/problem_positions', 10)

        self.center_map: Dict[str, list] = {}
        self.views: Dict[str, np.ndarray] = {}
        self.last_update_time: Dict[str, float] = {}

    def generate_template_points(self, left, right, segments=10):
        return [
            (
                int((1 - t) * left[0] + t * right[0]),
                int((1 - t) * left[1] + t * right[1])
            )
            for i in range(1, segments)
            for t in [i / segments]
        ]

    def center_callback(self, msg):
        try:
            data = json.loads(msg.data)
            part_id = data["id"]
            centers = data["centers"]
            self.center_map[part_id] = centers
        except Exception as e:
            self.get_logger().error(f"❌ 圓心資料解析錯誤: {e}")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame_info = msg.header.frame_id
            part_id = frame_info.split(":")[0]
            status = frame_info.split(":")[1] if ":" in frame_info else "unknown"
            key = f"{part_id}:{status}"

            h, w = frame.shape[:2]
            left = (0, h // 2)
            right = (w - 1, h // 2)
            template_points = self.generate_template_points(left, right)

            centers = self.center_map.get(part_id, [])
            tolerance = 20

            image = frame.copy()
            problem_indices = []

            for i, (tx, ty) in enumerate(template_points):
                closest = min(centers, key=lambda pt: abs(pt[0] - tx)) if centers else None
                if not closest or abs(closest[0] - tx) > tolerance:
                    cv2.circle(image, (tx, ty), 10, (0, 0, 255), 2)
                    problem_indices.append(i + 1)  # 位置從 1 開始

            # 發布問題孔位資訊
            if problem_indices:
                msg_out = {
                    "id": part_id,
                    "problem_indices": problem_indices
                }
                self.problem_pub.publish(String(data=json.dumps(msg_out)))
                self.get_logger().info(f"🚨 {part_id} 有異常孔位: {problem_indices}")

            resized = cv2.resize(image, (500, 150))
            hole_status = "normal" if not problem_indices else "abnormal"
            label = f"{part_id} [{hole_status}]"
            cv2.putText(resized, label, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0) if hole_status == "normal" else (0, 0, 255), 2)

            self.views[key] = resized
            self.last_update_time[key] = time.time()

            now = time.time()
            expired = [k for k, t in self.last_update_time.items() if now - t > 0.15]
            for k in expired:
                self.views.pop(k)
                self.last_update_time.pop(k)

            sorted_imgs = [img for _, img in sorted(self.views.items())]
            if sorted_imgs:
                combined = np.vstack(sorted_imgs)
                cv2.imshow("Hole Template Match", combined)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"❌ 模板匹配與標註錯誤: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HoleTemplateMatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 手動結束 HoleTemplateMatcher")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
