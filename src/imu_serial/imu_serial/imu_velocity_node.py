import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np

class ImuVelocityNode(Node):
    def __init__(self):
        super().__init__('imu_velocity_node')
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10)
        self.publisher = self.create_publisher(Odometry, 'odometry/imu_velocity', 10)

        self.prev_time = None
        self.velocity = np.zeros(3)  # x, y, z
        self.alpha = 0.8  # low-pass filter weight
        self.prev_acc = np.zeros(3)

    def imu_callback(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = current_time - self.prev_time
        if dt <= 0 or dt > 1.0:
            self.get_logger().warn(f'⚠️ 非法 dt: {dt:.3f}s，略過此次資料')
            self.prev_time = current_time
            return

        self.prev_time = current_time

        acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # 去除 z 向加速度（假設僅在平面移動）
        acc[2] = 0.0

        # low-pass filter
        acc_filtered = self.alpha * self.prev_acc + (1 - self.alpha) * acc
        self.prev_acc = acc_filtered

        # 閾值過濾（防止誤積分）
        if np.linalg.norm(acc_filtered) < 0.05:
            return

        # 積分速度
        self.velocity += acc_filtered * dt

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.twist.twist.linear.x = self.velocity[0]
        odom.twist.twist.linear.y = self.velocity[1]
        odom.twist.twist.linear.z = 0.0

        odom.twist.covariance[0] = 0.1  # x
        odom.twist.covariance[7] = 0.1  # y
        odom.twist.covariance[14] = 99999  # z 無效

        self.publisher.publish(odom)

        self.get_logger().info(
            f'✅ Vx: {self.velocity[0]:.2f} m/s, Vy: {self.velocity[1]:.2f} m/s, dt={dt:.3f}s')

def main(args=None):
    rclpy.init(args=args)
    node = ImuVelocityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
