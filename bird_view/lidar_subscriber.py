import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import math
import open3d as o3d


def rotate_points(points, angle_rad):
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    rot_mat = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
    return points @ rot_mat.T


def ransac_ground_segmentation(points, distance_threshold=0.1, ransac_n=3, num_iterations=1000):
    """
    Performs RANSAC plane segmentation to separate ground points.
    :param points: Nx3 numpy array
    :return: ground_flags (Nx1 array: 1.0 for ground, 0.0 for non-ground), plane_model coefficients
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold,
                                             ransac_n=ransac_n,
                                             num_iterations=num_iterations)

    ground_flags = np.zeros(points.shape[0], dtype=np.float32)
    ground_flags[inliers] = 1.0

    return ground_flags, plane_model


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.get_logger().info("LiDAR Subscriber Node has started.")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            '/sensing/lidar/top/pointcloud',
            self.listener_callback,
            qos_profile
        )

        self.publisher = self.create_publisher(PointCloud2, '/lidar/pointcloud_with_ground_flag', 10)

    def listener_callback(self, msg):
        # Read xyz points from PointCloud2
        points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array([[p[0], p[1], p[2]] for p in points_gen], dtype=np.float32)

        if points.shape[0] == 0:
            self.get_logger().warn("Received empty point cloud")
            return

        # Optional: Rotate points if needed (commented out)
        # points[:, :2] = rotate_points(points[:, :2], math.radians(180))

        # Run RANSAC ground segmentation
        ground_flags, plane_model = ransac_ground_segmentation(points)

        # Combine points and ground flags (add ground_flag as a 4th channel)
        points_with_flags = np.hstack((points, ground_flags[:, np.newaxis]))

        # Build new PointCloud2 message
        header = msg.header  # keep original header (timestamp, frame_id)

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='ground_flag', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # Flatten data to list of tuples for pc2.create_cloud
        points_list = [tuple(p) for p in points_with_flags]

        pc2_msg = pc2.create_cloud(header, fields, points_list)

        self.publisher.publish(pc2_msg)

        self.get_logger().info(f"Published point cloud with {len(points)} points and ground flags")


def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
