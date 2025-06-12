# lidar_bird_view.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt
from threading import Thread
import time

class LidarBirdView(Node):
    def __init__(self):
        super().__init__('lidar_bird_view')
        self.get_logger().info("LiDAR Bird View Node has started.")
        self.latest_points = np.empty((0, 2))

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/lidar/xy_points',
            self.listener_callback,
            10
        )

        # Start visualization thread
        self.running = True
        self.vis_thread = Thread(target=self.visualization_loop, daemon=True)
        self.vis_thread.start()

    def listener_callback(self, msg):
        data = np.array(msg.data, dtype=np.float32).reshape(-1, 2)
        self.latest_points = data

    def visualization_loop(self):
        plt.ion()
        fig, ax = plt.subplots()
        sc = ax.scatter([], [], s=1)
        ax.set_title("Bird's Eye LiDAR View")
        ax.set_xlabel("X (forward)")
        ax.set_ylabel("Y (left/right)")
        ax.set_xlim(-20, 20)
        ax.set_ylim(-20, 20)
        ax.grid(True)

        while self.running:
            sc.set_offsets(self.latest_points)
            fig.canvas.draw()
            fig.canvas.flush_events()
            time.sleep(0.1)  # 10 Hz update

    def destroy_node(self):
        self.running = False
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LidarBirdView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
