import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ScanSchedulerNode(Node):

    def __init__(self):
        super().__init__('scan_scheduler_node')

        # ===============================
        # GLOBAL PARAMETERS
        # ===============================
        self.declare_parameter('conveyor_speed', 100.0)            # mm/s
        self.declare_parameter('conveyor_acceleration', 0.0)       # mm/s²
        self.declare_parameter('fov_height_mm', 240.0)
        self.declare_parameter('overlap_ratio', 0.8)

        # Topics (configurable)
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/camera/filtered_image')

        self.v = self.get_parameter('conveyor_speed').value
        self.a = self.get_parameter('conveyor_acceleration').value
        self.fov_mm = self.get_parameter('fov_height_mm').value
        self.overlap = self.get_parameter('overlap_ratio').value

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        # ===============================
        # THRESHOLD
        # ===============================
        self.threshold_mm = self.fov_mm * self.overlap

        # ===============================
        # STATE
        # ===============================
        self.start_time = None
        self.last_scan_distance = 0.0

        # ===============================
        # ROS SETUP
        # ===============================
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10
        )

        self.publisher_ = self.create_publisher(
            Image,
            self.output_topic,
            10
        )

        self.get_logger().info(
            f"Scan Scheduler Started | Threshold: {self.threshold_mm:.2f} mm | "
            f"Input: {self.input_topic} → Output: {self.output_topic}"
        )


    # ===============================
    # DISTANCE (USING ROS TIME)
    # ===============================
    def compute_distance(self, current_time):

        if self.start_time is None:
            self.start_time = current_time
            return 0.0

        t = current_time - self.start_time

        return self.v * t + 0.5 * self.a * (t ** 2)


    # ===============================
    # CALLBACK
    # ===============================
    def image_callback(self, msg):

        # Convert ROS timestamp to seconds
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        current_distance = self.compute_distance(current_time)
        delta = current_distance - self.last_scan_distance

        if delta >= self.threshold_mm:

            # Forward frame
            self.publisher_.publish(msg)

            self.last_scan_distance = current_distance

            self.get_logger().info(
                f"Scan triggered | moved: {delta:.2f} mm"
            )

        # else → skip frame


def main(args=None):
    rclpy.init(args=args)
    node = ScanSchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()