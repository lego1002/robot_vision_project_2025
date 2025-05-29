import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np


class ImuBasedOdomNode(Node):
    def __init__(self):
        super().__init__('imu_based_odom_node')
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )
        self.publisher = self.create_publisher(Odometry, 'odometry/imu_velocity', 10)

        self.prev_time = None
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)

        self.alpha = 0.8  # low-pass filter
        self.prev_acc = np.zeros(3)

    def imu_callback(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = current_time - self.prev_time
        self.prev_time = current_time

        if dt <= 0 or dt > 1.0:
            self.get_logger().warn(f'⚠️ 非法 imu dt: {dt:.3f}s，略過此次資料')
            return

        acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            0.0  # 假設只在平面運動
        ])

        # 濾波
        acc_filtered = self.alpha * self.prev_acc + (1 - self.alpha) * acc
        self.prev_acc = acc_filtered

        # 閾值過濾
        if np.linalg.norm(acc_filtered) < 0.05:
            return

        # 積分速度與位置
        self.velocity += acc_filtered * dt
        self.position += self.velocity * dt

        # 發布 Odometry
        odom = Odometry()
        stamp = self.get_clock().now().to_msg()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.position.z = 0.0

        odom.twist.twist.linear.x = self.velocity[0]
        odom.twist.twist.linear.y = self.velocity[1]
        odom.twist.twist.linear.z = 0.0

        self.publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = ImuBasedOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()