import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import math
import time
from rich.console import Console
from rich.table import Table


class ImuSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_node')
        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)

        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            time.sleep(2)  # 等 Arduino 開機完成
            self.get_logger().info("Serial port opened successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("imu_serial_node is running.")

    def timer_callback(self):
        try:
            line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
            if not line.startswith('A:'):
                return

            # 解析資料
            line = line.replace('A:', '').replace(' G:', ',')
            parts = [int(x) for x in line.split(',')]
            if len(parts) != 6:
                self.get_logger().warn(f"Invalid data length: {line}")
                return

            ax, ay, az, gx, gy, gz = parts

            # 建立 Imu 訊息
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'

            # raw -> m/s^2
            msg.linear_acceleration.x = ax / 16384.0 * 9.80665
            msg.linear_acceleration.y = ay / 16384.0 * 9.80665
            msg.linear_acceleration.z = az / 16384.0 * 9.80665

            # raw -> rad/s
            msg.angular_velocity.x = gx / 131.0 * math.pi / 180
            msg.angular_velocity.y = gy / 131.0 * math.pi / 180
            msg.angular_velocity.z = gz / 131.0 * math.pi / 180
            
            
            # Create console object
            console = Console()

            # Assuming the values are coming from the msg object, we can directly use the data
            aX = round(msg.linear_acceleration.x, 2)
            aY = round(msg.linear_acceleration.y, 2)
            aZ = round(msg.linear_acceleration.z, 2)

            gX = round(msg.angular_velocity.x, 2)
            gY = round(msg.angular_velocity.y, 2)
            gZ = round(msg.angular_velocity.z, 2)

            # Create table object
            table = Table(show_header=True, header_style="bold magenta")
            table.add_column("Parameter", style="dim", width=20)
            table.add_column("Value", justify="right")

            # Add rows with data
            table.add_row("aX", str(aX))
            table.add_row("aY", str(aY))
            table.add_row("aZ", str(aZ))
            table.add_row("gX", str(gX))
            table.add_row("gY", str(gY))
            table.add_row("gZ", str(gZ))

            # Display the table
            console.print(table)

            self.publisher_.publish(msg)
            #self.get_logger().info(
                #f"IMU data: A=({aX},{aY},{aZ}) G=({gX},{gY},{gZ})")

        except Exception as e:
            self.get_logger().warn(f'Error reading serial: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImuSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()