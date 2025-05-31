import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import websocket
import numpy as np
import cv2
import threading

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.publisher = self.create_publisher(Image, '/camera/image_real', 10)
        self.bridge = CvBridge()

        # 連接到 WebSocket 伺服器
        self.ws = websocket.WebSocketApp("ws://localhost:1880/camera",
                                         on_message=self.on_message)
        self.get_logger().info("WebSocket Client Started")
        threading.Thread(target=self.ws.run_forever, daemon=True).start()

    def on_message(self, ws, message):
        try:
            # 將收到的 WebSocket 訊息轉換為影像
            np_arr = np.frombuffer(message, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # 轉換為 ROS2 影像訊息並發佈
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)

            self.get_logger().info("Published Image to /camera/image_real")
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down WebSocket client")
    finally:
        node.ws.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

