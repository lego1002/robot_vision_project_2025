from mpu6050 import mpu6050  # 加入此行
# 其餘 import 保留
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time
from rich.console import Console
from rich.table import Table
from rich.live import Live
import numpy as np

class ImuSerialNode(Node):
    def __init__(self, console, live):
        super().__init__('imu_serial_node')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.console = console
        self.live = live

        self.alpha = 0.8  # low-pass filter weight
        self.prev_acc = np.zeros(3)

        try:
            self.sensor = mpu6050(0x68)  # 使用 I2C 初始化 MPU6050
            self.get_logger().info("✅ 成功初始化 MPU6050 (I2C)")
        except Exception as e:
            self.get_logger().error(f"❌ MPU6050 初始化失敗: {e}")
            return

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("🚀 imu_serial_node 節點已啟動")

    def timer_callback(self):
        try:
            accel_raw = self.sensor.get_accel_data()
            gyro_raw = self.sensor.get_gyro_data()

            acc = np.array([accel_raw['x'], accel_raw['y'], accel_raw['z']]) * 9.80665
            acc_filtered = self.alpha * self.prev_acc + (1 - self.alpha) * acc
            self.prev_acc = acc_filtered

            if np.linalg.norm(acc_filtered) < 0.05:
                return

            gyro = np.array([gyro_raw['x'], gyro_raw['y'], gyro_raw['z']]) * math.pi / 180

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'
            msg.linear_acceleration.x = acc_filtered[0]
            msg.linear_acceleration.y = acc_filtered[1]
            msg.linear_acceleration.z = acc_filtered[2]
            msg.angular_velocity.x = gyro[0]
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = gyro[2]

            self.publisher_.publish(msg)

            table = Table(show_header=True, header_style="bold magenta")
            table.add_column("參數", style="dim", width=10)
            table.add_column("數值", justify="right")
            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            table.add_row("時間", f"{stamp_sec:.2f}")
            table.add_row("加速度X", f"{acc_filtered[0]:.2f}")
            table.add_row("加速度Y", f"{acc_filtered[1]:.2f}")
            table.add_row("加速度Z", f"{acc_filtered[2]:.2f}")
            table.add_row("角速度X", f"{gyro[0]:.2f}")
            table.add_row("角速度Y", f"{gyro[1]:.2f}")
            table.add_row("角速度Z", f"{gyro[2]:.2f}")

            self.live.update(table)

        except Exception as e:
            self.get_logger().warn(f'⚠️ I2C 讀取錯誤: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    console = Console()
    empty_table = Table(show_header=True, header_style="bold magenta")
    empty_table.add_column("參數", style="dim", width=10)
    empty_table.add_column("數值", justify="right")

    with Live(empty_table, console=console, refresh_per_second=10) as live:
        node = ImuSerialNode(console, live)
        rclpy.spin(node)
        node.destroy_node()
        
    rclpy.shutdown()