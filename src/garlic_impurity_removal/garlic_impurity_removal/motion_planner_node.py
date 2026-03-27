import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PointStamped
import time
import heapq
import math


class MotionPlannerNode(Node):
    """
    Receives world-frame detections (mm) and schedules pick commands so the
    delta hand arrives at (X, pickup_y) exactly when the impurity does.

    Coordinate contract (matches coordinate_node):
        Y = 0   → robot pick line  (where hand picks)
        Y = 150 → camera position  (upstream, new detections appear here)

    The belt moves in the -Y direction (garlic travels from camera toward robot).
    So travel distance for a detection at Y_world is simply Y_world - pickup_y.

    IMPORTANT: do NOT apply cam_to_robot offset here. coordinate_node already
    encodes it in the homography so Y_world=150 IS the camera position.
    """

    def __init__(self):
        super().__init__('motion_planner_node')

        # ===============================
        # PARAMETERS
        # ===============================
        self.declare_parameter('conveyor_speed',        1000.0)   # mm/s
        self.declare_parameter('conveyor_acceleration', 0.0)      # mm/s²
        self.declare_parameter('pickup_y',              100.0)    # mm — Y of pick line
        self.declare_parameter('x_max',                 60.0)     # mm
        self.declare_parameter('number_of_hands',       1)

        # Adaptive timing
        self.declare_parameter('robot_delay',     0.15)    # s — initial arm latency estimate
        self.declare_parameter('learning_rate',   0.0005)  # correction per failed pick

        # Duplicate suppression
        self.declare_parameter('duplicate_distance', 15.0)  # mm in X

        self.v          = self.get_parameter('conveyor_speed').value
        self.a          = self.get_parameter('conveyor_acceleration').value
        self.pickup_y   = self.get_parameter('pickup_y').value
        self.x_max      = self.get_parameter('x_max').value
        self.num_hands  = self.get_parameter('number_of_hands').value
        self.robot_delay   = self.get_parameter('robot_delay').value
        self.learning_rate = self.get_parameter('learning_rate').value
        self.dup_thresh    = self.get_parameter('duplicate_distance').value

        # ===============================
        # ROS SETUP
        # ===============================
        self.subscription = self.create_subscription(
            PoseArray,
            '/garlic_world_coordinates',
            self.receive_detections,
            10
        )

        self.feedback_sub = self.create_subscription(
            PointStamped,
            '/robot/pick_feedback',
            self.handle_feedback,
            10
        )

        self.publisher = self.create_publisher(
            PointStamped,
            '/robot/target',
            10
        )

        # ===============================
        # INTERNAL STATE
        # ===============================
        self.target_queue   = []   # min-heap: (priority, execute_time, target_dict)
        self.last_picks     = []   # for feedback matching
        self.recent_targets = []   # (X, timestamp) for duplicate suppression

        self.timer = self.create_timer(0.01, self.process_targets)

        self.get_logger().info(
            f"Motion Planner Ready | v={self.v} mm/s | "
            f"pickup_y={self.pickup_y} mm | delay={self.robot_delay:.3f}s"
        )

    # ===============================
    # TRAVEL TIME CALCULATION
    # ===============================
    def compute_travel_time(self, distance_mm):
        """
        Time for an object at `distance_mm` ahead of the pick line to arrive.

        Constant speed:    t = d / v
        Accelerating belt: solve  d = v*t + 0.5*a*t²
                           → 0.5a*t² + v*t - d = 0
                           → take positive root
        """
        if distance_mm <= 0:
            return 0.0

        if abs(self.a) < 1e-6:
            return distance_mm / self.v

        A = 0.5 * self.a
        B = self.v
        C = -distance_mm

        discriminant = B * B - 4 * A * C
        if discriminant < 0:
            return None

        t1 = (-B + math.sqrt(discriminant)) / (2 * A)
        t2 = (-B - math.sqrt(discriminant)) / (2 * A)

        candidates = [t for t in (t1, t2) if t > 0]
        return min(candidates) if candidates else None

    # ===============================
    # DUPLICATE FILTER
    # ===============================
    def is_duplicate(self, x):
        now = time.time()
        return any(
            abs(px - x) < self.dup_thresh and (now - t) < 2.0
            for px, t in self.recent_targets
        )

    # ===============================
    # RECEIVE DETECTIONS
    # ===============================
    def receive_detections(self, msg):

        now = time.time()

        for pose in msg.poses:

            X       = pose.position.x   # mm, already clamped by coordinate_node
            Y_world = pose.position.y   # mm, where 0=pick-line, 150=camera

            # Distance the impurity still needs to travel to reach the pick line
            distance = Y_world - self.pickup_y

            if distance <= 0:
                # Already past the pick line — too late
                self.get_logger().warn(
                    f"Detection at Y={Y_world:.1f} already past pickup_y={self.pickup_y:.1f}, skip"
                )
                continue

            X = max(-self.x_max, min(self.x_max, X))

            if self.is_duplicate(X):
                continue

            travel_time = self.compute_travel_time(distance)

            if travel_time is None:
                self.get_logger().warn(f"Could not compute travel time for d={distance:.1f}")
                continue

            # Trigger the arm robot_delay seconds BEFORE the impurity arrives
            execute_time = now + travel_time - self.robot_delay

            # Tie-break: prefer targets closer to X=0 (shorter arm travel)
            priority = execute_time + abs(X) * 0.001

            heapq.heappush(self.target_queue, (
                priority,
                execute_time,
                {"x": X, "stamp": msg.header.stamp}
            ))

            self.recent_targets.append((X, now))
            if len(self.recent_targets) > 100:
                self.recent_targets.pop(0)

            self.get_logger().info(
                f"Queued X={X:.1f} | d={distance:.0f}mm | "
                f"travel={travel_time:.3f}s | delay={self.robot_delay:.3f}s"
            )

    # ===============================
    # EXECUTION LOOP  (10 ms tick)
    # ===============================
    def process_targets(self):

        now      = time.time()
        executed = 0

        while self.target_queue and executed < self.num_hands:

            priority, execute_time, target = self.target_queue[0]

            if now < execute_time:
                break

            heapq.heappop(self.target_queue)

            msg              = PointStamped()
            msg.header.stamp = target["stamp"]
            msg.header.frame_id = "world"
            msg.point.x      = float(target["x"])
            msg.point.y      = float(self.pickup_y)
            msg.point.z      = 0.0

            self.publisher.publish(msg)

            self.last_picks.append({"x": target["x"], "time": now})
            if len(self.last_picks) > 50:
                self.last_picks.pop(0)

            self.get_logger().info(f"EXECUTE PICK -> X={target['x']:.1f}")
            executed += 1

    # ===============================
    # FEEDBACK & ADAPTIVE LEARNING
    # ===============================
    def handle_feedback(self, msg):

        fx      = msg.point.x
        success = msg.point.z   # 1.0 = success, 0.0 = miss

        # Match feedback to the closest recent pick
        closest = min(self.last_picks, key=lambda p: abs(p["x"] - fx), default=None)
        if closest is None:
            return

        # On a miss: nudge robot_delay slightly earlier (more lead time)
        # On success: no change (stable)
        if success != 1.0:
            self.robot_delay = max(0.0, min(0.5, self.robot_delay - self.learning_rate * 2.0))
            self.get_logger().warn(
                f"Pick miss -> robot_delay adjusted to {self.robot_delay:.4f}s"
            )
        else:
            self.get_logger().info(f"Pick success | delay={self.robot_delay:.4f}s")


def main(args=None):
    rclpy.init(args=args)
    node = MotionPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()