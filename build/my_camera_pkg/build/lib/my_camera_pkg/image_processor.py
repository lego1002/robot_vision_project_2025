import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')

        self.bridge = CvBridge()

        # 訂閱原始影像
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10)

        # 發布處理後的影像
        self.publisher = self.create_publisher(
            Image,
            '/camera/image_processed',
            10)

        self.get_logger().info('🎯 ImageProcessor 已啟動，等待影像資料...')

    def callback(self, msg):
        try:
            # 將 ROS Image 轉為 OpenCV 格式
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # ✅ 在這裡做影像處理：轉為灰階
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 顯示畫面（開發時可用，部署時可關掉）
            cv2.imshow("Processed (Gray)", gray)
            cv2.waitKey(1)

            # 將灰階圖轉為 ROS Image（mono8）
            processed_msg = self.bridge.cv2_to_imgmsg(gray, encoding='mono8')
            processed_msg.header = msg.header  # 保留原時間戳與 frame_id

            # 發布處理後的影像
            self.publisher.publish(processed_msg)
            self.get_logger().info('📤 已發布處理後影像 /camera/image_processed')

        except Exception as e:
            self.get_logger().error(f"❌ 處理影像失敗: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 手動中止 ImageProcessor")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

