import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Pose, PoseArray
from cv_bridge import CvBridge
import numpy as np
import cv2
import math
from ultralytics import YOLO
from scipy.stats import zscore
from tf2_ros import Buffer, TransformListener, LookupException, TimeoutException
import tf_transformations
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import sensor_msgs_py.point_cloud2 as pc2


class CameraLidarFusion(Node):
    def __init__(self):
        super().__init__('camera_lidar_fusion')
        self.bridge = CvBridge()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.image_sub = self.create_subscription(
            Image, '/sensing/camera/front/image_raw',
            self.image_callback, qos_profile)

        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar/pointcloud_with_ground_flag',
            self.lidar_callback, qos_profile)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.model = YOLO('yolov8m.pt')
        self.out_pub = self.create_publisher(PoseArray, '/detected_vehicles', 10)

        self.fx, self.fy = 500, 500
        self.cx, self.cy = 320, 240

        self.K = np.array([
            [533.0181026793728, 0.0, 533.1950304735122],
            [0.0, 484.917807487239, 309.95867583935154],
            [0.0, 0.0, 1.0]
        ])
        self.D = np.array([
            -0.30854049349822915,
            0.08268565804376049,
            0.0005477275652276282,
            -0.0003941952306063375,
            0.0
        ])
        self.image_size = (960, 600)

        self.new_K, _ = cv2.getOptimalNewCameraMatrix(self.K, self.D, self.image_size, alpha=0)
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.K, self.D, None, self.new_K, self.image_size, cv2.CV_16SC2
        )

        self.lidar_points = []
        self.get_logger().info('CameraLidarFusion started')

    def lidar_callback(self, msg: PointCloud2):
        points = []
        for pt in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([pt[0], pt[1], pt[2]])
        self.lidar_points = np.array(points, dtype=np.float64)

    def project_lidar_to_image(self, lidar_points, stamp):
        if len(lidar_points) == 0:
            return np.empty((0, 2), dtype=int), np.empty((0, 3))

        try:
            tf_lidar_to_base = self.tf_buffer.lookup_transform(
                'sensor_kit_base_link',
                'lidar_top_link',
                stamp,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            tf_base_to_camera = self.tf_buffer.lookup_transform(
                'camera_front_optical_link',
                'sensor_kit_base_link',
                stamp,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except (LookupException, TimeoutException) as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return np.empty((0, 2), dtype=int), np.empty((0, 3))

        t_lidar = np.array([
            tf_lidar_to_base.transform.translation.x,
            tf_lidar_to_base.transform.translation.y,
            tf_lidar_to_base.transform.translation.z
        ])
        q_lidar = [
            tf_lidar_to_base.transform.rotation.x,
            tf_lidar_to_base.transform.rotation.y,
            tf_lidar_to_base.transform.rotation.z,
            tf_lidar_to_base.transform.rotation.w,
        ]

        t_camera = np.array([
            tf_base_to_camera.transform.translation.x,
            tf_base_to_camera.transform.translation.y,
            tf_base_to_camera.transform.translation.z
        ])
        q_camera = [
            tf_base_to_camera.transform.rotation.x,
            tf_base_to_camera.transform.rotation.y,
            tf_base_to_camera.transform.rotation.z,
            tf_base_to_camera.transform.rotation.w,
        ]

        rot_lidar = tf_transformations.quaternion_matrix(q_lidar)[:3, :3]
        rot_camera = tf_transformations.quaternion_matrix(q_camera)[:3, :3]

        points_base = (rot_lidar @ lidar_points.T).T + t_lidar
        points_camera = (rot_camera @ points_base.T).T + t_camera
        R_z_180 = np.array([
            [-1, 0, 0],
            [0, 1, 0],
            [0, 0, -1]
        ])
        points_camera = (R_z_180 @ points_camera.T).T

        img_pts = []
        world_pts_camera = []

        for x, y, z in points_camera:
            if z <= 0.1 or math.isnan(z) or math.isinf(z):
                continue

            u = int((x * self.fx) / z + self.cx)
            v = int((y * self.fy) / z + self.cy)

            if 0 <= u < 960 and 0 <= v < 600:
                img_pts.append((u, v))
                world_pts_camera.append((x, y, z))

        img_pts = np.array(img_pts)
        world_pts_camera = np.array(world_pts_camera)

        if len(world_pts_camera) == 0:
            return np.empty((0, 2), dtype=int), np.empty((0, 3))

        valid = world_pts_camera[:, 2] > -1.5
        img_pts = img_pts[valid]
        world_pts_camera = world_pts_camera[valid]

        return img_pts, world_pts_camera

    def image_callback(self, msg: Image):
        raw_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        undistorted_img = cv2.remap(raw_img, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
        img = cv2.resize(undistorted_img, (640, 640))

        results = self.model(img)
        img_pts, world_pts = self.project_lidar_to_image(self.lidar_points, msg.header.stamp)

        # Visualize projected LiDAR points
        #for pt in img_pts:
            #cv2.circle(img, (int(pt[0]), int(pt[1])), 2, (0, 255, 0), -1)

        poses_msg = PoseArray()
        poses_msg.header.frame_id = 'camera_front'
        poses_msg.header.stamp = msg.header.stamp

        for res in results:
            boxes = res.boxes.xyxy.cpu().numpy()
            classes = res.boxes.cls.cpu().numpy().astype(int)

            for box, cls in zip(boxes, classes):
                if cls not in [2, 3, 5, 7]:
                    continue

                x1, y1, x2, y2 = box.astype(int)
                center_u = int((x1 + x2) / 2)
                center_v = int((y1 + y2) / 2)

                if len(img_pts) == 0:
                    continue

                in_box_mask = (
                        (img_pts[:, 0] >= x1) & (img_pts[:, 0] <= x2) &
                        (img_pts[:, 1] >= y1) & (img_pts[:, 1] <= y2)
                )
                points_in_box = world_pts[in_box_mask]

                if len(points_in_box) == 0:
                    continue

                distances = np.linalg.norm(points_in_box, axis=1)
                mean = np.mean(distances)
                std = np.std(distances)

                inliers = np.abs(distances - mean) < 1.5 * std

                if np.sum(inliers) == 0:
                    continue

                filtered_points = points_in_box[inliers]
                centroid = np.mean(filtered_points, axis=0)
                distance = np.linalg.norm(centroid)

                yaw = 0.0
                q = tf_transformations.quaternion_from_euler(0, 0, yaw)

                pose = Pose()
                pose.position.x = float(centroid[0])
                pose.position.y = float(centroid[1])
                pose.position.z = float(centroid[2])
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                poses_msg.poses.append(pose)

                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.circle(img, (center_u, center_v), 5, (0, 255, 0), -1)

                color = (0, 255, 0) if distance < 10 else (0, 255, 255) if distance < 20 else (0, 0, 255)
                cv2.putText(img, f"{distance:.1f}m", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        self.out_pub.publish(poses_msg)
        cv2.imshow('Camera + LiDAR Fusion', img)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = CameraLidarFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
