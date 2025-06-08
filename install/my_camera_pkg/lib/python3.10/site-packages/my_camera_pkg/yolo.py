import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os

class YOLOPartClassifier(Node):
    def __init__(self):
        super().__init__('yolo_part_classifier')
        self.bridge = CvBridge()

        base_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(base_dir, 'yolo_model', 'best.pt')
        self.model = YOLO(model_path)

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_rect',
            self.callback,
            10)

        self.count_pub = self.create_publisher(Int32, '/yolo/part_count', 10)
        self.result_pub = self.create_publisher(String, '/yolo/part_result', 10)
        self.good_pub = self.create_publisher(Image, '/yolo/good_parts', 10)
        self.bad_pub = self.create_publisher(Image, '/yolo/bad_parts', 10)
        
        # 用與hsv做結合 偵測非此區物件
        self.center_pub = self.create_publisher(String, '/yolo/part_centers', 10)

        self.get_logger().info('🚀 YOLOPartClassifier 啟動，等待零件圖像...')

    def callback(self, msg):
        try:
            part_id = msg.header.frame_id
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            original_img = img.copy()
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            results = self.model(img_rgb, verbose=False)[0]
            #results = self.model.predict(img_rgb, imgsz=640, verbose=False)[0]

            if results.boxes is None or len(results.boxes) == 0:
                self.get_logger().info(f"❌ [{part_id}] 無偵測結果")
                return

            part_count = len(results.boxes)
            count_msg = Int32()
            count_msg.data = part_count
            self.count_pub.publish(count_msg)
            self.get_logger().info(f"📦 YOLO 偵測到零件數量：{part_count}")
            
            h, w, _ = img.shape
            
            cropped_images = []
            
            boxes_sorted = sorted(results.boxes, key=lambda b: float(b.xyxy[0][1]))
            
            center_list = []

            for i, box in enumerate(boxes_sorted):
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = self.model.names[cls_id]
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                status = "good" if conf >= 0.8 else "bad"

                pad = 20
                x1_pad = max(x1 - pad, 0)
                y1_pad = max(y1 - pad, 0)
                x2_pad = min(x2 + pad, w)
                y2_pad = min(y2 + pad, h)

                part_crop = original_img[y1_pad:y2_pad, x1_pad:x2_pad]

                # 建立 ROS 訊息
                crop_msg = self.bridge.cv2_to_imgmsg(part_crop, encoding='bgr8')
                crop_msg.header = msg.header
                crop_msg.header.frame_id = f"part_{part_id}_{i}:{status}"

                if status == "good":
                    self.good_pub.publish(crop_msg)
                    # 只有 good 才計算中心點
                    cx = int((x1 + x2) // 2)
                    cy = int((y1 + y2) // 2)
                    center_list.append((cx, cy))
                else:
                    self.bad_pub.publish(crop_msg)
                    
                # 標註圖像（不發布這個版本）
                crop_disp = cv2.resize(part_crop, (600, 150))
                label_text = f"{i} - {status}"
                cv2.putText(crop_disp, label_text, (5, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 0) if status == "good" else (0, 0, 255), 2)
                cropped_images.append(crop_disp)

                # 原圖畫框
                text = f"{label}: {conf:.2f} {status}"
                cv2.rectangle(img, (x1_pad, y1_pad), (x2_pad, y2_pad), (0, 255, 0), 2)
                cv2.putText(img, text, (x1_pad, y1_pad - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                result_msg = String()
                result_msg.data = f"{part_id}:{label}:{conf:.2f}:{status}"
                self.result_pub.publish(result_msg)
                self.get_logger().info(f"🔍 [{part_id}] 偵測到：{text}")

            center_msg = String()
            center_msg.data = str(center_list)
            self.center_pub.publish(center_msg)

            # 合併所有縮圖橫向排列（或可改為格子排）
            if cropped_images:
                combined_view = np.vstack(cropped_images)
                cv2.imshow("YOLO Crops Combined", combined_view)

            # yolo圖框圖像顯示
            display_width = 1080
            scale = display_width / img.shape[1]
            resized_img = cv2.resize(img, (display_width, int(img.shape[0] * scale)))
            cv2.imshow(f"YOLO - {part_id}", resized_img)
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