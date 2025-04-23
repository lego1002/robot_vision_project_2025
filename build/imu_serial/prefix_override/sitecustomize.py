import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jacky/Documents/robot_vision_project_2025/install/imu_serial'
