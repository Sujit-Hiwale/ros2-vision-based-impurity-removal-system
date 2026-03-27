import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
import cv2
import os
from ament_index_python.packages import get_package_share_directory


class CoordinateNode(Node):
    """
    Converts pixel detections to world coordinates (mm) using the homography
    matrix produced by calibrate_homography.py.

    Coordinate contract:
      Pixel frame  — origin top-left, +X right, +Y down
      World frame  — origin at robot pick-line, +X right (robot axis),
                     +Y toward camera (i.e. upstream on belt).
                     Y=0  → pick line
                     Y=150 → directly under camera  (= camera_to_robot_distance)

    The homography already encodes this layout because calibrate_homography
    maps dst_points as:
        BL=(-60,150)  BR=(60,150)  TR=(60,0)  TL=(-60,0)
    So after transformation:
        Y_world=0   → robot pick line  (bottom of conveyor FOV)
        Y_world=150 → camera position  (top of conveyor FOV)

    IMPORTANT: motion_planner must NOT apply cam_to_robot offset again.
    """

    def __init__(self):
        super().__init__('coordinate_node')

        # ===============================
        # PARAMETERS
        # ===============================
        self.declare_parameter('x_max', 60.0)
        self.declare_parameter('debug', False)

        self.x_max = self.get_parameter('x_max').value
        self.debug  = self.get_parameter('debug').value

        # ===============================
        # LOAD HOMOGRAPHY
        # ===============================
        pkg_path    = get_package_share_directory('garlic_impurity_removal')
        matrix_path = os.path.join(pkg_path, 'config', 'homography_matrix.npy')

        if not os.path.exists(matrix_path):
            self.get_logger().error("Homography file not found!")
            raise RuntimeError("Missing homography_matrix.npy")

        self.H = np.load(matrix_path)

        # Derive Y_max from homography:  transform the top-centre pixel of the
        # image to world coords — the resulting Y is the conveyor FOV height in mm.
        # This is equivalent to camera_to_robot_distance but read directly from the
        # calibration rather than requiring it as a separate parameter.
        self._derive_world_bounds()

        self.get_logger().info(
            f"Homography loaded | X: [{-self.x_max:.0f}, {self.x_max:.0f}] mm | "
            f"Y: [0, {self.y_max:.0f}] mm"
        )

        # ===============================
        # ROS SETUP
        # ===============================
        self.subscription = self.create_subscription(
            PoseArray,
            '/garlic_tracked_coordinates',
            self.convert_coordinates,
            10
        )

        self.publisher = self.create_publisher(
            PoseArray,
            '/garlic_world_coordinates',
            10
        )

    # ===============================
    # DERIVE BOUNDS FROM HOMOGRAPHY
    # ===============================
    def _derive_world_bounds(self):
        """
        Find the world-space Y extent by transforming the four dst_point
        corners back from the known calibration world points.
        Calibration always maps to [-60,0]..[60,150] world space.
        We read y_max directly from the homography dst layout.
        """
        # The calibration script always uses these dst corners:
        #   BL=(-60,150)  BR=(60,150)  TR=(60,0)  TL=(-60,0)
        # So y_max is always 150 mm (= cam_to_robot).
        # We verify this by probing a pixel near top-centre of a nominal frame.
        test_pixel = np.array([[[640.0, 0.0]]], dtype=np.float32)
        world      = cv2.perspectiveTransform(test_pixel, self.H)
        self.y_max = float(abs(world[0][0][1]))
        # Clamp to a sane range in case of a degenerate matrix
        self.y_max = max(50.0, min(self.y_max, 1000.0))

    # ===============================
    # CONVERSION CALLBACK
    # ===============================
    def convert_coordinates(self, msg):

        output_msg             = PoseArray()
        output_msg.header      = msg.header
        output_msg.header.frame_id = "world"

        for pose in msg.poses:

            px   = float(pose.position.x)
            py   = float(pose.position.y)
            area = float(pose.position.z)

            # Pixel → world (mm) via homography
            world_point = cv2.perspectiveTransform(
                np.array([[[px, py]]], dtype=np.float32), self.H
            )

            X_mm = float(world_point[0][0][0])
            Y_mm = float(world_point[0][0][1])

            if self.debug:
                self.get_logger().info(
                    f"PIX({px:.0f},{py:.0f}) -> WORLD({X_mm:.1f},{Y_mm:.1f})"
                )

            # Clamp X to robot workspace
            X_clamped = max(-self.x_max, min(self.x_max, X_mm))

            if self.debug and X_clamped != X_mm:
                self.get_logger().warn(f"X clamped: {X_mm:.1f} -> {X_clamped:.1f}")

            # Drop anything with Y outside [0, y_max]
            if not (0.0 <= Y_mm <= self.y_max):
                if self.debug:
                    self.get_logger().warn(f"Y out of range: {Y_mm:.1f}, dropping")
                continue

            world_pose             = Pose()
            world_pose.position.x  = X_clamped
            world_pose.position.y  = Y_mm
            world_pose.position.z  = area
            output_msg.poses.append(world_pose)

        if output_msg.poses:
            self.publisher.publish(output_msg)

        if self.debug:
            self.get_logger().info(
                f"Published {len(output_msg.poses)} world points"
            )


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()