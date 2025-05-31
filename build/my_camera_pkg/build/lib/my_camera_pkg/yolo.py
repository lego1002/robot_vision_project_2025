import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os

class YOLOPartClassifier(Node):
    def __init__(self):
        super().__init__('yolo_part_classifier')
        self.bridge = CvBridge()

        # 載入訓練好的 YOLO 模型
        base_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(base_dir, 'yolo_model', 'best.pt')
        self.model = YOLO(model_path)

        # 訂閱裁切後的零件圖
        self.subscription = self.create_subscription(
            Image,
            #'/parts/all_parts',
            '/camera/image_raw',
            self.callback,
            10)

        # 發布結果（信心值）
        self.result_pub = self.create_publisher(String, '/yolo/part_result', 10)

        self.get_logger().info('🚀 YOLOPartClassifier 啟動，等待零件圖像...')

    def callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            # YOLOv8 推論
            results = self.model(img_rgb, verbose=False)[0]

            if results.boxes is None or len(results.boxes) == 0:
                self.get_logger().info("❌ 無偵測結果")
                return

            for box in results.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = self.model.names[cls_id]
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # 🔲 畫框
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # 🏷️ 畫 label + 信心值
                text = f"{label}: {conf:.2f}"
                cv2.putText(img, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # 發布結果
                result_msg = String()
                result_msg.data = text
                self.result_pub.publish(result_msg)

                self.get_logger().info(f"🔍 偵測到：{text}")

            # 顯示影像
            cv2.imshow("YOLO Detection", img)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"❌ YOLO 推論錯誤: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YOLOPartClassifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 手動結束 YOLO 推論節點")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()