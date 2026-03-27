import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
import time


class TrackingNode(Node):
    """
    Deduplicates pixel-space detections across frames using spatial proximity
    and a TTL-based memory.

    Operates entirely in pixel space — no coordinate conversion here.
    Conversion happens downstream in coordinate_node.

    Input:  /garlic_pixel_detections  (PoseArray, pixel coords in x/y, area in z)
    Output: /garlic_tracked_coordinates (PoseArray, same schema, duplicates removed)
    """

    def __init__(self):
        super().__init__('tracking_node')

        # ===============================
        # PARAMETERS
        # ===============================
        self.declare_parameter('duplicate_threshold_pixels', 20.0)
        self.declare_parameter('tracking_ttl', 1.0)

        self.threshold = self.get_parameter('duplicate_threshold_pixels').value
        self.ttl       = self.get_parameter('tracking_ttl').value

        # ===============================
        # ROS SETUP
        # ===============================
        self.subscription = self.create_subscription(
            PoseArray,
            '/garlic_pixel_detections',
            self.track_objects,
            10
        )

        self.publisher = self.create_publisher(
            PoseArray,
            '/garlic_tracked_coordinates',
            10
        )

        # Memory: list of (x, y, timestamp)
        self.tracked_points = []

        self.get_logger().info(
            f"Tracking Node Ready | dup_threshold={self.threshold}px | TTL={self.ttl}s"
        )

    # ===============================
    def cleanup_tracks(self):
        now = time.time()
        self.tracked_points = [
            (x, y, t) for (x, y, t) in self.tracked_points
            if (now - t) < self.ttl
        ]

    def is_duplicate(self, x, y):
        return any(
            np.hypot(px - x, py - y) < self.threshold
            for px, py, _ in self.tracked_points
        )

    # ===============================
    def track_objects(self, msg):

        self.cleanup_tracks()

        output_msg             = PoseArray()
        output_msg.header      = msg.header
        output_msg.header.frame_id = "camera"

        now       = time.time()
        new_count = 0

        for pose in msg.poses:

            x = float(pose.position.x)
            y = float(pose.position.y)
            z = float(pose.position.z)

            if self.is_duplicate(x, y):
                continue

            self.tracked_points.append((x, y, now))

            new_pose             = Pose()
            new_pose.position.x  = x
            new_pose.position.y  = y
            new_pose.position.z  = z
            output_msg.poses.append(new_pose)
            new_count += 1

        if new_count > 0:
            self.publisher.publish(output_msg)

        self.get_logger().debug(
            f"Tracked {new_count} new | Active: {len(self.tracked_points)}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()