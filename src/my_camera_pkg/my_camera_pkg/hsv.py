import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os

class HSVFilterNode(Node):
    def __init__(self):
        super().__init__('hsv_filter_node')
        self.bridge = CvBridge()

        # 讀取 HSV 設定值
        CONFIG_PATH = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "hsv_config.json"
        )

        try:
            with open(CONFIG_PATH, "r") as f:
                config = json.load(f)
                self.lower = np.array([config['minH'], config['minS'], config['minV']])
                self.upper = np.array([config['maxH'], config['maxS'], config['maxV']])
                self.get_logger().info(f"✅ 成功讀取 HSV 設定：{self.lower} ~ {self.upper}")
        except Exception as e:
            self.get_logger().error(f"❌ 讀取 HSV 設定檔失敗: {e}")
            self.lower = np.array([0, 0, 0])
            self.upper = np.array([179, 255, 255])

        # 訂閱原始影像
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)

        # 發布切割後的零件集影像（作為 list）
        self.parts_publisher = self.create_publisher(Image, '/parts/all_parts', 10)

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower, self.upper)
            result = cv2.bitwise_and(frame, frame, mask=mask)

            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))
            thick = cv2.dilate(mask, kernel, iterations=1)
            contours, _ = cv2.findContours(thick, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            part_id = 0
            cropped_parts = []

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 2000:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                margin = 10
                x_pad = max(x - margin, 0)
                y_pad = max(y - margin, 0)
                x2_pad = min(x + w + margin, frame.shape[1])
                y2_pad = min(y + h + margin, frame.shape[0])

                cv2.rectangle(result, (x_pad, y_pad), (x2_pad, y2_pad), (0, 255, 0), 2)
                cv2.putText(result, f"Part {part_id}", (x_pad, y_pad - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

                crop = frame[y_pad:y2_pad, x_pad:x2_pad]
                cropped_parts.append((part_id, crop))
                part_id += 1

            # 顯示處理流程畫面
            hsv_bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            combined = np.hstack((frame, mask_bgr, result))
            cv2.imshow("HSV Segment + Parts", combined)

            # 顯示每個零件原圖裁切結果（各自一個視窗）
            for pid, crop in cropped_parts:
                cv2.imshow(f"Part {pid}", crop)

            cv2.waitKey(1)

            # 發布所有分割零件（逐個）
            for pid, crop in cropped_parts:
                msg_out = self.bridge.cv2_to_imgmsg(crop, encoding='bgr8')
                msg_out.header = msg.header
                self.parts_publisher.publish(msg_out)

        except Exception as e:
            self.get_logger().error(f"❌ HSV 篩選錯誤: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HSVFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 結束 hsv_filter_node")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
