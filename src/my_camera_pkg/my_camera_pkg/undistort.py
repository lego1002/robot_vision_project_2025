import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os

class FisheyeUndistortNode(Node):
    def __init__(self):
        super().__init__('fisheye_undistort_node')

        # 讀取 JSON 校正參數
        calib_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fisheye_calib_data.json")
        with open(calib_path, "r") as f:
            calib = json.load(f)

        self.K = np.array(calib["K"])
        self.D = np.array(calib["D"])
        self.P = np.array(calib["P"])
        self.scale_logical_size = tuple(calib["img_shape"])  # 校正時用的影像大小 (w, h)

        self.map1 = None
        self.map2 = None
        self.last_size = None

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Image, '/camera/image_rect', 10)

        self.get_logger().info("Fisheye undistort節點已啟動")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = frame.shape[:2]

            # 解析度變了就重新建 map
            if self.map1 is None or (w, h) != self.last_size:
                self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                    self.K, self.D, np.eye(3), self.P, (w, h), cv2.CV_16SC2)
                self.last_size = (w, h)
                self.get_logger().info(f"重新建立 undistort map for resolution: {w}x{h}")

            # 去失真
            undistorted = cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)

            # 發布去失真影像
            out_msg = self.bridge.cv2_to_imgmsg(undistorted, encoding='bgr8')
            out_msg.header = msg.header  # 保留時間戳與 frame_id
            self.pub.publish(out_msg)

            # 顯示原圖與去失真圖（左右拼接）
            # combined = np.hstack((
            #     cv2.resize(frame, (w, h)),
            #     cv2.resize(undistorted, (w, h))
            # ))
            # cv2.imshow("Original | Undistorted", combined)
            # cv2.waitKey(1)

            #  顯示yolo圖框影像
            # cv2.namedWindow("Undistorted", cv2.WINDOW_NORMAL)
            # cv2.imshow("Undistorted", undistorted)
            # cv2.resizeWindow("Undistorted", 1600 , 900)
            # cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"❌ 處理影像失敗: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FisheyeUndistortNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()
