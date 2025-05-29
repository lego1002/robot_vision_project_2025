import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
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
            self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            time.sleep(2)
            self.get_logger().info("✅ imu串列埠開啟成功")
        except Exception as e:
            self.get_logger().error(f"❌ imu串列埠開啟失敗: {e}")
            return

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("🚀 imu_serial_node 節點已啟動")

    def timer_callback(self):
        try:
            line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
            if not line.startswith('A:'):
                return
            
            line = line.replace('A:', '').replace(' G:', ',')
            parts = [int(x) for x in line.split(',')]
            if len(parts) != 6:
                self.get_logger().warn(f"⚠️ /imu/data 資料長度錯誤: {line}")
                return

            ax, ay, az, gx, gy, gz = parts

            # 原始加速度
            acc_raw = np.array([ax, ay, az]) / 16384.0 * 9.80665

            # low-pass filter
            acc_filtered = self.alpha * self.prev_acc + (1 - self.alpha) * acc_raw
            self.prev_acc = acc_filtered

            # 閾值濾波（雜訊太小不發佈）
            if np.linalg.norm(acc_filtered) < 0.05:
                return

            # 角速度_單位轉換（°/s → rad/s）
            gyro = np.array([gx, gy, gz]) / 131.0 * math.pi / 180

            # 建立 IMU 訊息
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

            # 畫面更新
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
            self.get_logger().warn(f'⚠️ imu串列埠讀取錯誤: {e}')


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
    