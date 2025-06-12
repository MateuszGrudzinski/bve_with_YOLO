import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, PoseStamped
from pyqtgraph.opengl import GLMeshItem, MeshData
import numpy as np
from PyQt5 import QtWidgets
from PyQt5.QtCore import pyqtSignal, QObject
import pyqtgraph.opengl as gl
import trimesh
import pkg_resources
import tf_transformations
from tf_transformations import euler_from_quaternion
from tf2_ros import Buffer, TransformListener, LookupException, TimeoutException
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
import sensor_msgs_py.point_cloud2 as pc2


def tf2_geometry_msgs_do_transform_pose(pose_stamped, transform_stamped):
    import geometry_msgs.msg
    import numpy as np
    import tf_transformations

    p = pose_stamped.pose.position
    q = pose_stamped.pose.orientation
    pose_mat = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
    pose_mat[0][3] = p.x
    pose_mat[1][3] = p.y
    pose_mat[2][3] = p.z

    t = transform_stamped.transform.translation
    tr = transform_stamped.transform.rotation
    tf_mat = tf_transformations.quaternion_matrix([tr.x, tr.y, tr.z, tr.w])
    tf_mat[0][3] = t.x
    tf_mat[1][3] = t.y
    tf_mat[2][3] = t.z

    transformed_mat = np.dot(tf_mat, pose_mat)

    new_pos = geometry_msgs.msg.Point()
    new_pos.x = transformed_mat[0][3]
    new_pos.y = transformed_mat[1][3]
    new_pos.z = transformed_mat[2][3]

    new_quat = tf_transformations.quaternion_from_matrix(transformed_mat)

    new_orient = geometry_msgs.msg.Quaternion()
    new_orient.x = new_quat[0]
    new_orient.y = new_quat[1]
    new_orient.z = new_quat[2]
    new_orient.w = new_quat[3]

    transformed_pose = geometry_msgs.msg.Pose()
    transformed_pose.position = new_pos
    transformed_pose.orientation = new_orient

    return transformed_pose


class AutopilotDisplay(Node, QObject):
    update_vehicles_signal = pyqtSignal(list)

    def __init__(self):
        Node.__init__(self, 'autopilot_display')
        QObject.__init__(self)
        self.get_logger().info("Autopilot Display Node started.")

        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.update_vehicles_signal.connect(self.update_vehicle_models)

        # PyQt setup
        self.app = QtWidgets.QApplication(sys.argv)
        self.window = QtWidgets.QWidget()
        self.window.setWindowTitle('Autopilot 3D View')
        self.window.resize(800, 600)
        self.layout = QtWidgets.QVBoxLayout()
        self.window.setLayout(self.layout)

        self.view = gl.GLViewWidget()
        self.view.opts['distance'] = 20
        self.view.setCameraPosition(distance=30, elevation=45, azimuth=270)
        self.layout.addWidget(self.view)

        self.detected_cars = []

        # Add grid
        grid = gl.GLGridItem()
        grid.scale(1, 1, 1)
        self.view.addItem(grid)

        # Load car mesh
        self.car_model_path = pkg_resources.resource_filename('bird_view', 'resource/models/LowPolyFiatUNO.obj')
        self.car_meshdata = self.load_mesh(self.car_model_path)

        # Initial car at origin
        initial_car = GLMeshItem(meshdata=self.car_meshdata, smooth=True, color=(1, 0, 0, 1), drawEdges=False)
        initial_car.scale(1.0, 1.0, 1.0)
        initial_car.rotate(90, 1, 0, 0)
        initial_car.rotate(-90, 0, 0, 1)
        self.view.addItem(initial_car)

        # Initial dummy point cloud
        points = np.random.uniform(-10, 10, (1000, 3)).astype(np.float32)
        self.scatter = gl.GLScatterPlotItem(pos=points, color=(0, 1, 1, 1), size=2.0)
        self.view.addItem(self.scatter)

        # Coordinate axes
        self.add_coordinate_axes()

        # Subscribe to PointCloud2 for lidar
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar/pointcloud_with_ground_flag',
            self.listener_callback,
            10
        )

        self.vehicle_subscription = self.create_subscription(
            PoseArray,
            '/detected_vehicles',
            self.detected_vehicles_callback,
            10
        )

        self.window.show()

    def add_coordinate_axes(self):
        x_line = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [5, 0, 0]]), color=(1, 0, 0, 1), width=2)
        y_line = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 5, 0]]), color=(0, 1, 0, 1), width=2)
        z_line = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0, 2]]), color=(0, 0, 1, 1), width=2)
        self.view.addItem(x_line)
        self.view.addItem(y_line)
        self.view.addItem(z_line)

    def load_mesh(self, model_path):
        mesh_or_scene = trimesh.load(model_path)
        if isinstance(mesh_or_scene, trimesh.Scene):
            mesh = trimesh.util.concatenate(mesh_or_scene.dump())
        else:
            mesh = mesh_or_scene
        return MeshData(vertexes=mesh.vertices, faces=mesh.faces)

    def listener_callback(self, msg: PointCloud2):
        # Extract points and ground flags from PointCloud2
        self.get_logger().info("PointCloud2 message received.")
        points = []
        ground_flags = []

        for p in pc2.read_points(msg, field_names=("x", "y", "z", "ground_flag"), skip_nans=True):
            points.append([p[0], p[1], p[2]])
            ground_flags.append(p[3])

        points = np.array(points, dtype=np.float32)
        ground_flags = np.array(ground_flags, dtype=np.float32)

        if len(points) == 0:
            return

        colors = np.zeros((points.shape[0], 4), dtype=np.float32)
        colors[ground_flags == 1.0] = [0, 1, 0, 1]  # green ground points
        colors[ground_flags != 1.0] = [1, 0, 0, 1]  # red non-ground points

        self.scatter.setData(pos=points, color=colors, size=2.0)

    def transform_pose(self, pose, from_frame, to_frame, stamp):
        ps = PoseStamped()
        ps.pose = pose
        ps.header.frame_id = from_frame
        ps.header.stamp = stamp

        try:
            transform = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                stamp,
                timeout=Duration(seconds=1.0)
            )
        except (LookupException, TimeoutException) as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

        transformed_ps = tf2_geometry_msgs_do_transform_pose(ps, transform)
        return transformed_ps

    def detected_vehicles_callback(self, msg: PoseArray):
        transformed_poses = []
        for pose in msg.poses:
            transformed_pose = self.transform_pose(pose, 'camera_front_optical_link', 'sensor_kit_base_link', msg.header.stamp)
            if transformed_pose is None:
                self.get_logger().warn("Pose transform failed, skipping vehicle")
                continue
            transformed_poses.append(transformed_pose)

        self.update_vehicles_signal.emit(transformed_poses)

    def update_vehicle_models(self, transformed_poses):
        # Remove previous cars
        for car in self.detected_cars:
            self.view.removeItem(car)
        self.detected_cars.clear()

        for pose in transformed_poses:
            # Log for debugging
            self.get_logger().info(
                f"Vehicle Pose - x: {pose.position.x:.2f}, y: {pose.position.y:.2f}, z: {pose.position.z:.2f}"
            )

            # Create a new car mesh item
            car = GLMeshItem(meshdata=self.car_meshdata, smooth=True, color=(0, 1, 0, 1), drawEdges=False)

            # Reset transforms to avoid stacking
            car.resetTransform()

            # Scale model
            car.scale(0.5, 0.5, 0.5)

            # Align mesh Z-up to OpenGL
            car.rotate(90, 1, 0, 0)

            # Convert quaternion to Euler for yaw
            q = pose.orientation
            quat = [q.x, q.y, q.z, q.w]
            roll, pitch, yaw = euler_from_quaternion(quat)

            # Apply yaw rotation (around Z after the initial alignment)
            car.rotate(np.degrees(-yaw), 0, 0, 1)

            # Translate car to its position (adjust Z if necessary)
            ogl_x = -pose.position.x
            ogl_y = -pose.position.y
            ogl_z = pose.position.z - 0.5  # move model to sit on ground

            car.translate(ogl_x, ogl_y, ogl_z)

            # Add to view and track
            self.view.addItem(car)
            self.detected_cars.append(car)


def main(args=None):
    rclpy.init(args=args)
    node = AutopilotDisplay()

    from threading import Thread
    ros_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    sys.exit(node.app.exec())

    rclpy.shutdown()
    ros_thread.join()


if __name__ == '__main__':
    main()
