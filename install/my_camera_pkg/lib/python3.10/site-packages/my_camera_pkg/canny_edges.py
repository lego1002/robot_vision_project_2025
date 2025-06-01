import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np
from typing import Dict
import time


class ImageProcessor(Node):
    def __init__(self):
        super().__init__('canny_edges')
        self.bridge = CvBridge()

        self.edge_views: Dict[str, np.ndarray] = {}
        
        self.edge_views: Dict[str, np.ndarray] = {}
        self.last_update_time: Dict[str, float] = {}

        self.good_sub = self.create_subscription(Image, '/yolo/good_parts', self.callback_good, 10)
        self.bad_sub = self.create_subscription(Image, '/yolo/bad_parts', self.callback_bad, 10)

        self.good_pub = self.create_publisher(Image, '/edges/good_parts', 10)
        self.bad_pub = self.create_publisher(Image, '/edges/bad_parts', 10)

        self.get_logger().info('🎯 ImageProcessor 已啟動：執行 Canny 邊緣檢測...')

    def process_and_publish(self, msg, publisher):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            frame_info = msg.header.frame_id
            part_id = frame_info.split(":")[0]
            status = frame_info.split(":")[1] if ":" in frame_info else "unknown"
            key = f"{part_id}:{status}"

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            denoised = cv2.medianBlur(gray, 5)
            blurred = cv2.GaussianBlur(denoised, (27, 27), 1)
            sharp = cv2.addWeighted(denoised, 1.5, blurred, -1, 20)
            edges = cv2.Canny(sharp, 70, 150)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
            thick_edges = cv2.dilate(edges, kernel, iterations=1)

            # 顯示流程：gray → denoised → sharp → edges → thick_edges
            gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            denoised_bgr = cv2.cvtColor(denoised, cv2.COLOR_GRAY2BGR)
            sharp_bgr = cv2.cvtColor(sharp, cv2.COLOR_GRAY2BGR)
            edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            thick_bgr = cv2.cvtColor(thick_edges, cv2.COLOR_GRAY2BGR)

            stack = [gray_bgr, denoised_bgr, sharp_bgr, edges_bgr, thick_bgr]
            resized_stack = [cv2.resize(img, (300, 150)) for img in stack]
            combined = np.hstack(resized_stack)

            cv2.imshow("Edge Process Flow", combined)
            cv2.waitKey(1)

            # 儲存邊緣圖像（用於原本整合顯示）
            edge_color = cv2.cvtColor(thick_edges, cv2.COLOR_GRAY2BGR)
            resized = cv2.resize(edge_color, (500, 150))
            label = f"{part_id} [{status}]"
            cv2.putText(resized, label, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0) if status == "good" else (0, 0, 255), 2)
            self.edge_views[key] = resized
            self.last_update_time[key] = time.time()

            now = time.time()
            expired_keys = [k for k, t in self.last_update_time.items() if now - t > 0.15]
            for k in expired_keys:
                self.edge_views.pop(k)
                self.last_update_time.pop(k)

            # 發布邊緣圖像
            edge_msg = self.bridge.cv2_to_imgmsg(thick_edges, encoding='mono8')
            edge_msg.header = msg.header
            publisher.publish(edge_msg)
            self.get_logger().info(f'📤 已發布邊緣影像：{frame_info}')

        except Exception as e:
            self.get_logger().error(f"❌ 邊緣檢測錯誤: {e}")

    def callback_good(self, msg):
        self.process_and_publish(msg, self.good_pub)

    def callback_bad(self, msg):
        self.process_and_publish(msg, self.bad_pub)

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 手動中止 Image Edge Processor")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

