import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        # 嘗試開啟 /dev/video5 攝影機
        self.cap = cv2.VideoCapture('/dev/video0')
        if not self.cap.isOpened():
            self.get_logger().error("無法開啟攝影機！")
            return

        self.timer = self.create_timer(0.1, self.publish_frame)
        self.get_logger().info('📷 Camera Node 已啟動')

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(img_msg)
            self.get_logger().info("📤 發布影像")
        else:
            self.get_logger().warn("❌ 無法讀取影像")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("💤 關閉 Camera Node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

