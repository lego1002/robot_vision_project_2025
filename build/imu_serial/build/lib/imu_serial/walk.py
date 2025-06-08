import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math


class ImuBasedOdomNode(Node):
    def __init__(self):
        super().__init__('imu_based_odom_node')
        self.publisher = self.create_publisher(Odometry, 'odometry/imu_velocity', 10)

        self.timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.start_time = self.get_clock().now()
        self.prev_time = self.start_time

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        if dt <= 0.0:
            return

        # 模擬前進與後退的速度
        forward_speed = 0.1  # m/s 向前
        backward_speed = -0.1  # m/s 向後
        # 控制左右方向的運動
        self.velocity[0] = forward_speed  # 設定為0.1或-0.1 模擬前後運動
        self.velocity[1] = 0.0  # 不考慮左右方向

        self.position += self.velocity * dt

        # 發布 Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.position.z = 0.0

        odom.twist.twist.linear.x = self.velocity[0]
        odom.twist.twist.linear.y = self.velocity[1]
        odom.twist.twist.linear.z = 0.0

        self.publisher.publish(odom)

        # 發布 TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = self.position[0]
        tf_msg.transform.translation.y = self.position[1]
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.w = 1.0  # 無旋轉

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuBasedOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()