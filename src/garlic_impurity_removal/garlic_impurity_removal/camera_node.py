import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time


class USBCameraNode(Node):

    def __init__(self):
        super().__init__('usb_camera_node')

        # ===============================
        # PARAMETERS
        # ===============================
        self.declare_parameter('camera_index', 2)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('show_preview', False)
        self.declare_parameter('simulate', True)

        self.camera_index = self.get_parameter('camera_index').value
        self.width        = self.get_parameter('camera_width').value
        self.height       = self.get_parameter('camera_height').value
        self.fps          = self.get_parameter('camera_fps').value
        self.show_preview = self.get_parameter('show_preview').value
        self.simulate     = self.get_parameter('simulate').value

        # ===============================
        # ROS SETUP
        # ===============================
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge      = CvBridge()

        # ===============================
        # CAMERA / SIM INIT
        # ===============================
        self.cap = None

        if self.simulate:
            self.get_logger().warn("SIM: publishing synthetic frames")
            self._sim_frame_idx = 0
        else:
            self.init_camera()

        # ===============================
        # TIMER
        # ===============================
        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)

        self.last_time   = time.time()
        self.frame_count = 0

        self.get_logger().info(
            f"Camera Node Ready | simulate={self.simulate} | "
            f"{self.width}x{self.height} @ {self.fps} FPS"
        )

    # ===============================
    # CAMERA INITIALIZATION
    # ===============================
    def init_camera(self):

        if self.cap is not None:
            self.cap.release()

        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera {self.camera_index}")
            return

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS,          self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)

        for _ in range(5):
            self.cap.read()

        self.get_logger().info("Camera initialized (V4L2)")

    # ===============================
    # SYNTHETIC FRAME (SIMULATE MODE)
    # ===============================
    def make_sim_frame(self):
        """
        Produces a realistic-enough synthetic conveyor frame:
        - Gray belt background with subtle noise
        - A few randomly-placed white garlic blobs
        - A small dark impurity blob that drifts down the belt each frame
        """
        frame = np.full((self.height, self.width, 3), 160, dtype=np.uint8)

        # Belt texture noise
        noise = np.random.randint(0, 20, (self.height, self.width, 3), dtype=np.uint8)
        frame = cv2.add(frame, noise)

        # Garlic cloves (white ellipses)
        rng = np.random.default_rng(seed=42)
        for _ in range(12):
            cx = int(rng.integers(100, self.width - 100))
            cy = int(rng.integers(80, self.height - 80))
            cv2.ellipse(frame, (cx, cy), (30, 20), int(rng.integers(0, 180)),
                        0, 360, (230, 230, 215), -1)

        # Drifting impurity (dark blob) — moves down belt
        drift_y = int((self._sim_frame_idx * 8) % self.height)
        drift_x = self.width // 2 + 50
        cv2.ellipse(frame, (drift_x, drift_y), (12, 8), 20, 0, 360, (60, 45, 30), -1)

        self._sim_frame_idx += 1
        return frame

    # ===============================
    # MAIN LOOP
    # ===============================
    def timer_callback(self):

        if self.simulate:
            frame = self.make_sim_frame()
        else:
            if self.cap is None or not self.cap.isOpened():
                self.get_logger().warn("Camera lost, retrying...")
                self.init_camera()
                return

            ret, frame = self.cap.read()

            if not ret or frame is None:
                self.get_logger().warn("Frame read failed, retrying...")
                self.init_camera()
                return

        if self.show_preview:
            cv2.imshow("Camera Feed", frame)
            cv2.waitKey(1)

        msg             = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        self.publisher_.publish(msg)

        self.frame_count += 1

        if self.frame_count >= 30:
            current_time  = time.time()
            fps_actual    = self.frame_count / (current_time - self.last_time)
            self.get_logger().info(f"Camera FPS: {fps_actual:.2f}")
            self.frame_count = 0
            self.last_time   = current_time

    # ===============================
    def destroy_node(self):
        self.get_logger().info("Shutting down camera node...")
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        if self.show_preview:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = USBCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()