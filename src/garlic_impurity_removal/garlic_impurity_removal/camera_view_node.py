import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from ament_index_python.packages import get_package_share_directory

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


# Colours (BGR)
COL_ROI_BOUNDARY  = (0,   200, 255)   # amber — homography calibration quad
COL_PATCHCORE_ROI = (255, 140,   0)   # blue  — PatchCore crop region
COL_DETECTION     = (0,   255,   0)   # green — detected impurity
COL_TARGET        = (0,     0, 255)   # red   — scheduled pick target
COL_SUCCESS       = (0,   255, 255)   # yellow-green — successful pick
COL_FAIL          = (0,    80, 255)   # orange-red   — failed pick


class CameraViewNode(Node):

    def __init__(self):
        super().__init__('camera_view_node')

        self.bridge          = CvBridge()
        self.latest_frame    = None
        self.pixel_detections = []   # [(px, py, timestamp)]
        self.robot_targets   = []    # [(wx, wy)]  — world mm
        self.pick_feedback   = []    # [(wx, wy, success_float)]

        # ===============================
        # PARAMETERS
        # ===============================
        self.declare_parameter('image_topic',    '/camera/image_raw')
        self.declare_parameter('display_fps',    30)
        self.declare_parameter('enable_display', True)
        self.declare_parameter('data_ttl',       1.5)

        # ROI fractions used by patchcore_node — must match
        self.declare_parameter('roi_top',    0.2)
        self.declare_parameter('roi_bottom', 0.9)
        self.declare_parameter('roi_left',   0.1)
        self.declare_parameter('roi_right',  0.9)

        self.image_topic    = self.get_parameter('image_topic').value
        self.display_fps    = self.get_parameter('display_fps').value
        self.enable_display = self.get_parameter('enable_display').value
        self.ttl            = self.get_parameter('data_ttl').value

        self.roi_top    = self.get_parameter('roi_top').value
        self.roi_bottom = self.get_parameter('roi_bottom').value
        self.roi_left   = self.get_parameter('roi_left').value
        self.roi_right  = self.get_parameter('roi_right').value

        # ===============================
        # LOAD HOMOGRAPHY
        # ===============================
        pkg_path = get_package_share_directory('garlic_impurity_removal')
        H_path   = os.path.join(pkg_path, "config", "homography_matrix.npy")

        if os.path.exists(H_path):
            self.H     = np.load(H_path)
            self.H_inv = np.linalg.inv(self.H)
            self.get_logger().info("Homography loaded")
        else:
            self.H     = None
            self.H_inv = None
            self.get_logger().warn("No homography matrix found — boundary overlay disabled")

        # Pre-compute world-space boundary corners (matching calibrate_homography dst_points)
        # BL=(-60,150)  BR=(60,150)  TR=(60,0)  TL=(-60,0)
        self._world_boundary = np.array([
            [-60.0, 150.0],
            [ 60.0, 150.0],
            [ 60.0,   0.0],
            [-60.0,   0.0],
        ], dtype=np.float32)

        # ===============================
        # QoS  (match camera_node publisher)
        # ===============================
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(Image,        self.image_topic,            self.image_callback,     qos)
        self.create_subscription(PoseArray,    '/garlic_pixel_detections',  self.detection_callback, 10)
        self.create_subscription(PointStamped, '/robot/target',             self.target_callback,    10)
        self.create_subscription(PointStamped, '/robot/pick_feedback',      self.feedback_callback,  10)

        cv2.namedWindow("Garlic System Debug View", cv2.WINDOW_NORMAL)

        self.timer = self.create_timer(1.0 / self.display_fps, self.display)
        self.get_logger().info("Camera View Node started")

    # ===============================
    # CALLBACKS
    # ===============================
    def image_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def detection_callback(self, msg):
        self.pixel_detections = [
            (int(p.position.x), int(p.position.y), time.time())
            for p in msg.poses
        ]

    def target_callback(self, msg):
        self.robot_targets.append((msg.point.x, msg.point.y))
        self.robot_targets = self.robot_targets[-20:]

    def feedback_callback(self, msg):
        self.pick_feedback.append((msg.point.x, msg.point.y, msg.point.z))
        self.pick_feedback = self.pick_feedback[-20:]

    # ===============================
    # COORDINATE HELPERS
    # ===============================
    def world_to_pixel(self, x_mm, y_mm):
        """Convert world (mm) to pixel coordinates using inverse homography."""
        if self.H_inv is None:
            return None
        pt  = np.array([[[x_mm, y_mm]]], dtype=np.float32)
        px  = cv2.perspectiveTransform(pt, self.H_inv)
        return int(px[0][0][0]), int(px[0][0][1])

    def world_boundary_pixels(self):
        """
        Return the 4 pixel corners of the calibrated world region.
        These are the same 4 points the maintainer clicked during calibration,
        projected back via H_inv.
        """
        if self.H_inv is None:
            return None
        pts_w = self._world_boundary.reshape(1, -1, 2).astype(np.float32)
        pts_p = cv2.perspectiveTransform(pts_w, self.H_inv)
        return pts_p.reshape(-1, 2).astype(np.int32)

    # ===============================
    # DRAW HELPERS
    # ===============================
    def draw_roi_overlays(self, frame):

        corners = self.world_boundary_pixels()

        if corners is None:
            return

        # ---- Compute ROI from homography ----
        xs = corners[:, 0]
        ys = corners[:, 1]

        roi_x0 = int(min(xs))
        roi_x1 = int(max(xs))
        roi_y0 = int(min(ys))
        roi_y1 = int(max(ys))

        # ---- Draw PatchCore ROI (same as homography) ----
        pt1 = (roi_x0, roi_y0)
        pt2 = (roi_x1, roi_y1)

        cv2.rectangle(frame, pt1, pt2, COL_PATCHCORE_ROI, 1)
        cv2.putText(frame, "PatchCore ROI",
                    (pt1[0] + 4, pt1[1] + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, COL_PATCHCORE_ROI, 1)

        # ---- Draw Homography boundary ----
        cv2.polylines(frame, [corners], isClosed=True,
                    color=COL_ROI_BOUNDARY, thickness=2)

        labels = ["BL(-60,150)", "BR(60,150)", "TR(60,0)", "TL(-60,0)"]
        for i, (px, py) in enumerate(corners):
            cv2.circle(frame, (px, py), 5, COL_ROI_BOUNDARY, -1)
            cv2.putText(frame, labels[i],
                        (px + 4, py - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, COL_ROI_BOUNDARY, 1)
            
    def draw_detections(self, frame):
        """Draw detected impurities as green circles with a hand-assignment label."""
        for i, (px, py, _) in enumerate(self.pixel_detections):
            cv2.circle(frame, (px, py), 6, COL_DETECTION, -1)
            cv2.circle(frame, (px, py), 12, COL_DETECTION, 1)
            # Hand assignment — currently always H1 (single hand)
            cv2.putText(frame, "H1",
                        (px + 8, py - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, COL_DETECTION, 1)

    def draw_targets(self, frame):
        """Draw scheduled pick targets (world -> pixel) as red crosses."""
        for (wx, wy) in self.robot_targets:
            px = self.world_to_pixel(wx, wy)
            if px:
                cv2.drawMarker(frame, px, COL_TARGET, cv2.MARKER_CROSS, 18, 2)
                cv2.putText(frame, f"({wx:.0f},{wy:.0f})",
                            (px[0] + 6, px[1] - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, COL_TARGET, 1)

    def draw_feedback(self, frame):
        """Draw pick outcome circles (yellow-green = success, orange = fail)."""
        for (wx, wy, success) in self.pick_feedback:
            px = self.world_to_pixel(wx, wy)
            if px:
                color = COL_SUCCESS if success == 1.0 else COL_FAIL
                cv2.circle(frame, px, 14, color, 2)
                label = "OK" if success == 1.0 else "MISS"
                cv2.putText(frame, label,
                            (px[0] + 10, px[1] + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    def draw_legend(self, frame):
        items = [
            (COL_ROI_BOUNDARY,  "Homography boundary"),
            (COL_PATCHCORE_ROI, "PatchCore ROI"),
            (COL_DETECTION,     "Detection (H1)"),
            (COL_TARGET,        "Pick target"),
            (COL_SUCCESS,       "Pick success"),
            (COL_FAIL,          "Pick miss"),
        ]
        x0, y0 = 10, 16
        for i, (color, text) in enumerate(items):
            y = y0 + i * 18
            cv2.rectangle(frame, (x0, y - 9), (x0 + 14, y + 3), color, -1)
            cv2.putText(frame, text, (x0 + 18, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1)

    # ===============================
    # MAIN DISPLAY LOOP
    # ===============================
    def cleanup(self):
        now = time.time()
        self.pixel_detections = [
            (x, y, t) for (x, y, t) in self.pixel_detections
            if now - t < self.ttl
        ]

    def display(self):

        if not self.enable_display:
            return

        self.cleanup()

        if self.latest_frame is None:
            placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(placeholder, "Waiting for camera...",
                        (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 200, 200), 2)
            cv2.imshow("Garlic System Debug View", placeholder)
            cv2.waitKey(1)
            return

        frame = self.latest_frame.copy()

        self.draw_roi_overlays(frame)
        self.draw_detections(frame)
        self.draw_targets(frame)
        self.draw_feedback(frame)
        self.draw_legend(frame)

        cv2.imshow("Garlic System Debug View", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()