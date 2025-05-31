import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('canny_edges')

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
            '/camera/image_edges',
            10)

        self.get_logger().info('🎯 ImageProcessor 已啟動：執行 Canny 邊緣檢測...')

    def callback(self, msg):
        try:
            # 將 ROS Image 轉為 OpenCV 格式
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 轉灰階
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # median
            denoised = cv2.medianBlur(gray, 5)
            # GaussianBlur
            blurred = cv2.GaussianBlur(denoised, (27, 27), 1)
            # Unsharp Masking 銳利化
            sharp = cv2.addWeighted(denoised, 1.5, blurred, -1, 20)
            # Canny 邊緣檢測
            edges = cv2.Canny(sharp, 70, 150)
            # dilation
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
            thick_edges = cv2.dilate(edges, kernel, iterations=1)
            # morphological
            # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
            # morphological = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

            # 水平合併（一次最多建議三張）
            combined= np.hstack((denoised, blurred, sharp, edges, thick_edges))
            # 顯示合併後影像
            cv2.imshow("Processing Pipeline", combined)
            cv2.waitKey(1)

            # 將灰階圖轉為 ROS Image（mono8）
            edge_msg = self.bridge.cv2_to_imgmsg(thick_edges, encoding='mono8')
            edge_msg.header = msg.header # 保留原時間戳與 frame_id

            # 發布處理後的影像
            self.publisher.publish(edge_msg)
            self.get_logger().info('📤 已發布邊緣影像 /camera/image_edges')

        except Exception as e:
            self.get_logger().error(f"❌ 邊緣檢測錯誤: {e}")

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

