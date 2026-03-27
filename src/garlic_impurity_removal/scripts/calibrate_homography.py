import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory


def order_points(pts):
    """
    Order 4 clicked points as: BL, BR, TR, TL
    (matching the dst_points layout used in compute_homography).
    """
    pts = np.array(pts, dtype=np.float32)
    pts = pts[np.argsort(pts[:, 1])]   # sort by Y: top two, then bottom two
    top    = pts[:2][np.argsort(pts[:2, 0])]    # TL, TR
    bottom = pts[2:][np.argsort(pts[2:, 0])]    # BL, BR
    tl, tr = top
    bl, br = bottom
    return np.array([bl, br, tr, tl], dtype=np.float32)


def compute_warp_size(frame_w, frame_h):
    """
    Derive warp canvas size from input resolution.
    Rule: half of min(W, H), rounded to nearest 64, clamped to [128, 512].
    Examples:
        1280x720 → min=720 → 360 → nearest 64 = 384
        640x480  → min=480 → 240 → nearest 64 = 256
        1920x1080→ min=1080→ 540 → nearest 64 = 512 (clamped)
    """
    base = min(frame_w, frame_h) // 2
    size = round(base / 64) * 64
    return max(128, min(size, 512))


class CalibrationNode(Node):

    def __init__(self):
        super().__init__('calibration_node')

        self.bridge        = CvBridge()
        self.frame         = None
        self.pixel_points  = []
        self.warp_size     = None   # derived once first frame arrives

        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        cv2.namedWindow("Calibration")
        cv2.setMouseCallback("Calibration", self.click_event)

        self.get_logger().info("\n=== CALIBRATION MODE ===")
        self.get_logger().info("Click the 4 conveyor corners in any order:")
        self.get_logger().info("  Bottom-left, Bottom-right, Top-right, Top-left")
        self.get_logger().info("Keys:  r = reset clicks   q = quit\n")

    def image_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        if self.warp_size is None:
            h, w    = self.frame.shape[:2]
            self.warp_size = compute_warp_size(w, h)
            self.get_logger().info(
                f"Frame size: {w}x{h} -> warp canvas: {self.warp_size}x{self.warp_size}"
            )

        self.display()

    def click_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(self.pixel_points) < 4:
            self.pixel_points.append([x, y])
            self.get_logger().info(f"Point {len(self.pixel_points)}: pixel ({x}, {y})")

    def display(self):

        if self.frame is None:
            return

        frame = self.frame.copy()

        for i, (x, y) in enumerate(self.pixel_points):
            cv2.circle(frame, (int(x), int(y)), 6, (0, 255, 0), -1)
            cv2.putText(frame, str(i + 1), (int(x) + 8, int(y) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if len(self.pixel_points) == 4:
            pts = np.array(self.pixel_points, dtype=np.int32)
            cv2.polylines(frame, [pts], True, (255, 80, 0), 2)

        cv2.imshow("Calibration", frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('r'):
            self.pixel_points = []
            self.get_logger().info("Points reset")

        elif key == ord('q'):
            self.destroy_node()
            rclpy.shutdown()

        if len(self.pixel_points) == 4:
            self.compute_homography()

    def compute_homography(self):

        S = self.warp_size if self.warp_size else 512

        ordered_pts = order_points(self.pixel_points)

        self.get_logger().info(f"Ordered pixel points:\n{ordered_pts}")

        # World space (mm):
        #   BL=(-60,150)  BR=(60,150) — bottom = close to robot (pick line end)
        #   TR=(60,0)     TL=(-60,0)  — top    = far from robot (camera position)
        #
        # Y axis: 0 = robot pick line, 150 = camera footprint top edge
        # X axis: -60 = left, +60 = right
        dst_world = np.array([
            [-60.0, 150.0],   # BL
            [ 60.0, 150.0],   # BR
            [ 60.0,   0.0],   # TR
            [-60.0,   0.0],   # TL
        ], dtype=np.float32)

        H = cv2.getPerspectiveTransform(ordered_pts, dst_world)

        self.get_logger().info(f"Homography matrix (pixel -> mm):\n{H}")

        pkg_path    = get_package_share_directory('garlic_impurity_removal')
        config_path = os.path.join(pkg_path, "config")
        os.makedirs(config_path, exist_ok=True)

        H_path = os.path.join(config_path, "homography_matrix.npy")
        np.save(H_path, H)
        self.get_logger().info(f"Saved homography -> {H_path}")

        # Debug warp — maps same 4 corners to a square canvas for visual check
        dst_canvas = np.array([
            [0,   S  ],   # BL
            [S,   S  ],   # BR
            [S,   0  ],   # TR
            [0,   0  ],   # TL
        ], dtype=np.float32)

        H_canvas = cv2.getPerspectiveTransform(ordered_pts, dst_canvas)
        warped   = cv2.warpPerspective(self.frame, H_canvas, (S, S))

        cv2.imshow("Warp Preview", warped)
        cv2.waitKey(0)

        self.get_logger().info(
            f"Calibration complete | warp canvas {S}x{S}"
        )
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = CalibrationNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()