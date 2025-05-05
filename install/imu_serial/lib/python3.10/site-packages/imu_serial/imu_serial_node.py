import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import math
import time
from rich.console import Console
from rich.table import Table
from rich.live import Live  # 動態更新畫面
import numpy as np


class ImuSerialNode(Node):
    def __init__(self, console, live):
        super().__init__('imu_serial_node')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.console = console
        self.live = live

        try:
            # 設定串列埠（注意 Uno 為 ttyACM0）
            self.serial_port = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
            time.sleep(2)  # 等 Arduino 開機完成
            self.get_logger().info("✅ 串列埠開啟成功")
        except Exception as e:
            self.get_logger().error(f"❌ 串列埠開啟失敗: {e}")
            return

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("🚀 imu_serial_node 節點已啟動")

    def timer_callback(self):
        try:
            # 讀取一行資料，忽略錯誤字元
            line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
            if not line.startswith('A:'):
                return

            # 解析字串格式
            line = line.replace('A:', '').replace(' G:', ',')
            parts = [int(x) for x in line.split(',')]
            if len(parts) != 6:
                self.get_logger().warn(f"⚠️ 資料長度錯誤: {line}")
                return

            ax, ay, az, gx, gy, gz = parts

            # 建立 IMU 訊息
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'

            # 加速度_單位轉換（g → m/s²）
            acc = np.array([ax, ay, az]) / 16384.0 * 9.80665
            msg.linear_acceleration.x = acc[0]
            msg.linear_acceleration.y = acc[1]
            msg.linear_acceleration.z = acc[2]

            # 角速度_單位轉換（°/s → rad/s）
            gyro = np.array([gx, gy, gz]) / 131.0 * math.pi / 180
            msg.angular_velocity.x = gyro[0]
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = gyro[2]
            
            
            self.publisher_.publish(msg)

            # 動態更新 table 資料
            table = Table(show_header=True, header_style="bold magenta")
            table.add_column("參數", style="dim", width=10)
            table.add_column("數值", justify="right")

            table.add_row("加速度X", f"{msg.linear_acceleration.x:.2f}")
            table.add_row("加速度Y", f"{msg.linear_acceleration.y:.2f}")
            table.add_row("加速度Z", f"{msg.linear_acceleration.z:.2f}")
            table.add_row("角速度X", f"{msg.angular_velocity.x:.2f}")
            table.add_row("角速度Y", f"{msg.angular_velocity.y:.2f}")
            table.add_row("角速度Z", f"{msg.angular_velocity.z:.2f}")

            self.live.update(table)

        except Exception as e:
            self.get_logger().warn(f'⚠️ 串列埠讀取錯誤: {e}')


def main(args=None):
    rclpy.init(args=args)

    # 建立 Rich 的 console 和 live table
    console = Console()
    empty_table = Table(show_header=True, header_style="bold magenta")
    empty_table.add_column("參數", style="dim", width=10)
    empty_table.add_column("數值", justify="right")

    with Live(empty_table, console=console, refresh_per_second=10) as live:
        node = ImuSerialNode(console, live)
        rclpy.spin(node)
        node.destroy_node()

    rclpy.shutdown()
