import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

class HoleDetector(Node):
    def __init__(self):
        super().__init__('hole_detector_node')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_edges',
            self.callback,
            10)

        self.publisher = self.create_publisher(
            Int32,
            '/hole_detector/hole_count',
            10)

        self.get_logger().info('🕳️ HoleDetector圓度分析啟動')

    def callback(self, msg):
        try:
            edges = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            frame = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            hole_count = 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 40 or area > 500:
                    continue

                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0:
                    continue

                # 圓度計算
                circularity = 4 * np.pi * area / (perimeter ** 2)
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = w / h if h != 0 else 0

                if circularity > 0.75 and 0.85 < aspect_ratio < 1.15:
                    hole_count += 1
                    center = (int(x + w / 2), int(y + h / 2))
                    radius = int((w + h) / 4)
                    cv2.circle(frame, center, radius, (0, 255, 0), 2)
                    cv2.putText(frame, str(hole_count), (center[0] - 5, center[1] - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # 顯示畫面（可關）
            cv2.imshow("Hole Detector", frame)
            cv2.waitKey(1)

            # 發布數量
            msg_out = Int32()
            msg_out.data = hole_count
            self.publisher.publish(msg_out)

            self.get_logger().info(f"🔵 偵測到孔洞數量: {hole_count}")

        except Exception as e:
            self.get_logger().error(f"❌ 孔洞偵測錯誤: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HoleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 手動關閉 HoleDetector")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
