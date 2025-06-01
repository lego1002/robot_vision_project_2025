import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from typing import Dict
import time
import json

class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')
        
        self.result = "尚未比對"
        self.hole_info_map: Dict[str, list] = {}
        self.hole_info_time: Dict[str, float] = {}

        self.create_subscription(String, '/analysis/part_difference', self.result_callback, 10)
        self.create_subscription(String, '/hole_template/problem_positions', self.hole_callback, 10)

        # Tkinter UI
        self.root = tk.Tk()
        self.root.title("零件檢查")
        
        self.root.geometry("300x200") 
        self.root.resizable(False, False)

        self.label_result = tk.Label(self.root, text="比對結果: 尚未比對", font=("Arial", 16))
        self.label_hole = tk.Label(self.root, text="異常孔洞: 無", font=("Arial", 16))

        self.label_result.pack()
        self.label_hole.pack()

        self.update_loop()

    def result_callback(self, msg):
        self.result = msg.data

    def hole_callback(self, msg):
        try:
            data = json.loads(msg.data)
            part_id = data.get("id", "未知")
            positions = data.get("problem_indices", [])
            if positions:
                self.hole_info_map[part_id] = positions
                self.hole_info_time[part_id] = time.time()
        except Exception as e:
            self.hole_info_map = {"解析錯誤": []}

    def update_loop(self):
        self.label_result.config(text=f"--零件情況--\n{self.result}")

        now = time.time()
        expired = [pid for pid, t in self.hole_info_time.items() if now - t > 0.15]
        for pid in expired:
            self.hole_info_map.pop(pid, None)
            self.hole_info_time.pop(pid, None)

        if self.hole_info_map:
            summary = "\n".join([f"{pid} → {idxs}" for pid, idxs in sorted(self.hole_info_map.items())])
        else:
            summary = "無"
        self.label_hole.config(text=f"\n--異常孔洞--\n{summary}")

        self.root.after(200, self.update_loop)

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        from threading import Thread
        spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
