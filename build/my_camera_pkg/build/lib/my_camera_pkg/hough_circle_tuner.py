import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os

# 儲存檔案路徑
CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hough_config.json")

class HoughCircleTuner(Node):
    def __init__(self):
        super().__init__('hough_circle_tuner')
        self.bridge = CvBridge()

        # 訂閱原始相機影像
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_rect',
            self.callback,
            10)

        # 初始化參數與GUI
        self.create_trackbars()
        self.get_logger().info('🔧 Hough Circle Tuner 已啟動！按 s 儲存設定')

    def create_trackbars(self):
        cv2.namedWindow("Hough Circle Tuner", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Hough Circle Tuner", 1960, 1080)
        
        cv2.namedWindow("Hough Circle Tuner")
        cv2.createTrackbar("dp x100", "Hough Circle Tuner", 12, 100, lambda x: None)   # 1.2
        cv2.createTrackbar("minDist", "Hough Circle Tuner", 20, 200, lambda x: None)
        cv2.createTrackbar("param1", "Hough Circle Tuner", 100, 300, lambda x: None)
        cv2.createTrackbar("param2", "Hough Circle Tuner", 22, 100, lambda x: None)
        cv2.createTrackbar("minRadius", "Hough Circle Tuner", 8, 100, lambda x: None)
        cv2.createTrackbar("maxRadius", "Hough Circle Tuner", 15, 100, lambda x: None)

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 處理流程（固定順序）
            denoised = cv2.medianBlur(gray, 5)
            blurred = cv2.GaussianBlur(denoised, (27, 27), 1)
            sharp = cv2.addWeighted(denoised, 1.5, blurred, -1, 20)
            edges = cv2.Canny(sharp, 70, 150)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
            thick_edges = cv2.dilate(edges, kernel, iterations=2)

            # 建立彩色顯示圖
            output = cv2.cvtColor(thick_edges, cv2.COLOR_GRAY2BGR)

            # 讀取滑桿參數
            dp = cv2.getTrackbarPos("dp x100", "Hough Circle Tuner") / 100.0
            minDist = cv2.getTrackbarPos("minDist", "Hough Circle Tuner")
            param1 = cv2.getTrackbarPos("param1", "Hough Circle Tuner")
            param2 = cv2.getTrackbarPos("param2", "Hough Circle Tuner")
            minRadius = cv2.getTrackbarPos("minRadius", "Hough Circle Tuner")
            maxRadius = cv2.getTrackbarPos("maxRadius", "Hough Circle Tuner")

            # 執行 Hough Circle 偵測
            circles = cv2.HoughCircles(
                thick_edges,
                cv2.HOUGH_GRADIENT,
                dp=dp,
                minDist=minDist,
                param1=param1,
                param2=param2,
                minRadius=minRadius,
                maxRadius=maxRadius
            )

            # 畫出結果
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i, (x, y, r) in enumerate(circles[0, :]):
                    cv2.circle(output, (x, y), r, (0, 255, 0), 2)
                    cv2.putText(output, str(i + 1), (x - 5, y - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # 顯示視窗
            cv2.imshow("Hough Circle Tuner", output)

            # 如果按下 s 鍵，儲存目前設定
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                params = {
                    "dp": dp,
                    "minDist": minDist,
                    "param1": param1,
                    "param2": param2,
                    "minRadius": minRadius,
                    "maxRadius": maxRadius
                }
                save_path = os.path.join(os.getcwd(), 'hough_config.json')
                with open(CONFIG_PATH, 'w') as f:
                    json.dump(params, f, indent=4)
                self.get_logger().info(f"💾 已儲存設定到 {CONFIG_PATH}")

        except Exception as e:
            self.get_logger().error(f"❌ Hough Circle Tuner 錯誤: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HoughCircleTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 手動關閉 Hough Circle Tuner")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
