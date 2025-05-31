import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

class HoughHoleDetector(Node):
    def __init__(self):
        super().__init__('hough_hole_detector')
        self.bridge = CvBridge()

        # 訂閱邊緣圖像
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_edges',
            self.callback,
            10)

        # 發布孔洞數量
        self.publisher = self.create_publisher(
            Int32,
            '/hole_detector/hole_count',
            10)

        self.get_logger().info('🟢 HoughHoleDetector 已啟動')

    def callback(self, msg):
        try:
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            output = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            # ⚙️ HoughCircles 參數
            circles = cv2.HoughCircles(
                gray,
                cv2.HOUGH_GRADIENT,
                dp=0.12,              # 影像解析度縮放比例
                minDist=20,          # 圓與圓間最小距離
                param1=100,          # Canny 高閾值
                param2=22,           # 圓形累加門檻（越小越鬆）
                minRadius=8,
                maxRadius=15
            )

            hole_count = 0
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i, (x, y, r) in enumerate(circles[0, :]):
                    cv2.circle(output, (x, y), r, (0, 255, 0), 2)
                    cv2.putText(output, str(i+1), (x-5, y-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    hole_count += 1

            # 顯示結果
            cv2.imshow("Hough Circle Detection", output)
            cv2.waitKey(1)

            # 發布數量
            msg_out = Int32()
            msg_out.data = hole_count
            self.publisher.publish(msg_out)
            self.get_logger().info(f"🔵 偵測到圓形孔洞數量: {hole_count}")

        except Exception as e:
            self.get_logger().error(f"❌ Hough 圓偵測錯誤: {e}")

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