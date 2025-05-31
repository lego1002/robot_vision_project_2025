import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class HoughCircleTuner(Node):
    def __init__(self):
        super().__init__('hough_circle_tuner')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_edges',
            self.callback,
            10)

        # 建立滑桿視窗
        self.create_trackbars()
        self.get_logger().info('🔧 Hough Circle Tuner 已啟動！')

    def create_trackbars(self):
        cv2.namedWindow("Hough Circle Tuner")

        cv2.createTrackbar("dp x10", "Hough Circle Tuner", 12, 30, lambda x: None)           # 1.2
        cv2.createTrackbar("minDist", "Hough Circle Tuner", 20, 100, lambda x: None)
        cv2.createTrackbar("param1", "Hough Circle Tuner", 100, 300, lambda x: None)
        cv2.createTrackbar("param2", "Hough Circle Tuner", 15, 100, lambda x: None)
        cv2.createTrackbar("minRadius", "Hough Circle Tuner", 3, 100, lambda x: None)
        cv2.createTrackbar("maxRadius", "Hough Circle Tuner", 15, 100, lambda x: None)

    def callback(self, msg):
        try:
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            output = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            # 讀取滑桿參數
            dp = cv2.getTrackbarPos("dp x10", "Hough Circle Tuner") / 100.0
            minDist = cv2.getTrackbarPos("minDist", "Hough Circle Tuner")
            param1 = cv2.getTrackbarPos("param1", "Hough Circle Tuner")
            param2 = cv2.getTrackbarPos("param2", "Hough Circle Tuner")
            minRadius = cv2.getTrackbarPos("minRadius", "Hough Circle Tuner")
            maxRadius = cv2.getTrackbarPos("maxRadius", "Hough Circle Tuner")

            # Hough Circle 偵測
            circles = cv2.HoughCircles(
                gray,
                cv2.HOUGH_GRADIENT,
                dp=dp,
                minDist=minDist,
                param1=param1,
                param2=param2,
                minRadius=minRadius,
                maxRadius=maxRadius
            )

            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i, (x, y, r) in enumerate(circles[0, :]):
                    cv2.circle(output, (x, y), r, (0, 255, 0), 2)
                    cv2.putText(output, str(i+1), (x-5, y-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            cv2.imshow("Hough Circle Tuner", output)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"❌ HoughCircle GUI 偵測錯誤: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HoughCircleTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 手動關閉 HoughCircle GUI")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
