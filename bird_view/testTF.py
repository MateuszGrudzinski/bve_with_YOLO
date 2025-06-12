import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, TimeoutException
from rclpy.time import Time

class TFChecker(Node):
    def __init__(self):
        super().__init__('tf_checker')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.check_tf)

    def check_tf(self):
        now = self.get_clock().now()
        try:
            # Example chain: camera_front -> sensor_kit_base_link
            cam_to_base = self.tf_buffer.lookup_transform(
                'sensor_kit_base_link',
                'camera0/camera_link',
                now,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            print('Camera to Base Link transform:', cam_to_base)

            # Base link -> velodyne_top
            base_to_lidar = self.tf_buffer.lookup_transform(
                'velodyne_top',
                'sensor_kit_base_link',
                now,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            print('Base Link to Velodyne transform:', base_to_lidar)

        except (LookupException, TimeoutException) as e:
            print(f'TF lookup failed: {e}')

def main():
    rclpy.init()
    node = TFChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
