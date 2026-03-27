import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import serial
import time
import random


class DeltaHandNode(Node):

    def __init__(self):
        super().__init__('delta_hand_node')

        # ===============================
        # PARAMETERS
        # ===============================
        self.declare_parameter('serial_port',     '/dev/ttyUSB0')
        self.declare_parameter('baud_rate',       115200)
        self.declare_parameter('x_max',           60.0)
        self.declare_parameter('pickup_z',        -40.0)    # mm — depth of pick stroke
        self.declare_parameter('number_of_hands', 1)
        self.declare_parameter('simulate',        True)

        # Arm motion timing — exposed so motion_planner can be tuned to match
        self.declare_parameter('move_time',       0.25)    # s — time per IK move
        self.declare_parameter('grip_time',       0.20)    # s — time to open/close gripper

        self.port       = self.get_parameter('serial_port').value
        self.baud       = self.get_parameter('baud_rate').value
        self.x_max      = self.get_parameter('x_max').value
        self.pickup_z   = self.get_parameter('pickup_z').value
        self.num_hands  = self.get_parameter('number_of_hands').value
        self.simulate   = self.get_parameter('simulate').value
        self.move_time  = self.get_parameter('move_time').value
        self.grip_time  = self.get_parameter('grip_time').value

        # total_time = 3 moves (approach, pick, retract) + grip + release
        # sequence: pre-approach → pick → grip → retract → home → release
        self.cycle_time = self.move_time * 4 + self.grip_time * 2

        # ===============================
        # SERIAL INIT
        # ===============================
        self.ser = None

        if not self.simulate:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=1)
                time.sleep(2)
                self.get_logger().info(f"Connected to {self.port}")

                self.send("SetGripperType 1")
                self.send("StopStepper")
                self.send("SetSpeed 1000")
                self.send("SetAcceleration 500")
                self.send("Forward")
                self.send("StartStepper")

            except Exception as e:
                self.get_logger().error(f"Serial error: {e} — falling back to SIM")
                self.simulate = True

        if self.simulate:
            self.get_logger().warn("Running in SIMULATION MODE — no serial output")

        # ===============================
        # ROS
        # ===============================
        self.subscription = self.create_subscription(
            PointStamped, "/robot/target", self.target_callback, 10
        )

        self.feedback_pub = self.create_publisher(
            PointStamped, "/robot/pick_feedback", 10
        )

        self.targets    = []
        self.busy_until = 0.0

        self.timer = self.create_timer(0.02, self.process_targets)

        self.get_logger().info(
            f"Delta Hand Ready | cycle_time={self.cycle_time:.2f}s | "
            f"pickup_z={self.pickup_z} mm"
        )

    # ===============================
    def send(self, cmd):
        if self.simulate:
            self.get_logger().info(f"[SIM] {cmd}")
            return
        try:
            self.ser.write((cmd + "\n").encode())
            self.ser.flush()
        except Exception as e:
            self.get_logger().error(f"Serial send error: {e}")

    # ===============================
    def target_callback(self, msg):

        x = msg.point.x
        y = msg.point.y

        if abs(x) > self.x_max:
            return

        # Duplicate filter (5 mm tolerance)
        if any(abs(t['x'] - x) < 5 for t in self.targets):
            return

        self.targets.append({"x": x, "y": y, "stamp": msg.header.stamp})
        self.targets = self.targets[-50:]

    # ===============================
    def process_targets(self):

        now = time.time()

        if now < self.busy_until:
            return

        if not self.targets:
            return

        # Execute the next queued target
        self.targets.sort(key=lambda t: t['y'])
        target = self.targets.pop(0)
        self.execute_pick(target)

    # ===============================
    def execute_pick(self, target):

        x = round(target['x'], 1)
        y = round(target['y'], 1)
        z = self.pickup_z

        self.get_logger().info(f"Picking -> X:{x}, Y:{y}")

        # Pick sequence:
        #   1. Move to safe height above target
        #   2. Descend to pick depth
        #   3. Grip
        #   4. Retract to safe height
        #   5. Return to home
        #   6. Release
        self.send(f"IK [{x},{y},-20]")
        time.sleep(self.move_time)

        self.send(f"IK [{x},{y},{z}]")
        time.sleep(self.move_time)

        self.send("grip")
        time.sleep(self.move_time)

        self.send(f"IK [{x},{y},-20]")
        time.sleep(self.move_time)

        self.send("IK [0,0,-40]")
        time.sleep(self.move_time)

        self.send("release")
        time.sleep(self.move_time)


        success = random.random() > 0.2 if self.simulate else True

        feedback             = PointStamped()
        feedback.header.stamp    = target["stamp"]
        feedback.header.frame_id = "world"
        feedback.point.x     = x
        feedback.point.y     = y
        feedback.point.z     = 1.0 if success else 0.0
        self.feedback_pub.publish(feedback)

        if success:
            self.get_logger().info("Pick success")
        else:
            self.get_logger().warn("Pick failed")

        self.busy_until = time.time() + self.cycle_time

    # ===============================
    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DeltaHandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()