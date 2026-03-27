from launch import LaunchDescription
from launch_ros.actions import Node

# ==============================================================================
#  SYSTEM LAUNCH — MAINTAINER CONFIGURATION
#
#  This is the ONLY file that needs editing between runs.
#  All values flow into nodes via ROS parameters — zero hardcoding in node files.
#
#  SECTIONS
#  --------
#  1.  MODE            — simulate / hardware toggle
#  2.  HARDWARE        — camera index, serial port
#  3.  CAMERA          — resolution, FPS
#  4.  CONVEYOR        — belt speed and acceleration
#  5.  ROBOT GEOMETRY  — workspace limits, pick depth
#  6.  VISION          — PatchCore sensitivity, ROI crop fractions
#  7.  TRACKING        — duplicate suppression, track lifetime
#  8.  MOTION PLANNER  — timing, adaptive learning
#  9.  DELTA HAND      — arm speed, gripper timing
#  10. DEBUG VIEW      — display options
# ==============================================================================


# ------------------------------------------------------------------------------
# 1.  MODE
#     SIMULATE = True  → no serial output, synthetic camera frames, Foxglove-safe
#     SIMULATE = False → real camera + real delta hand over serial
# ------------------------------------------------------------------------------
SIMULATE = False


# ------------------------------------------------------------------------------
# 2.  HARDWARE  (used only when SIMULATE = False)
# ------------------------------------------------------------------------------
CAMERA_INDEX = 2
SERIAL_PORT  = '/dev/ttyUSB0'
BAUD_RATE    = 115200


# ------------------------------------------------------------------------------
# 3.  CAMERA
# ------------------------------------------------------------------------------
CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480
CAMERA_FPS    = 30


# ------------------------------------------------------------------------------
# 4.  CONVEYOR
# ------------------------------------------------------------------------------
CONVEYOR_SPEED        = 1000.0   # mm/s
CONVEYOR_ACCELERATION = 0.0      # mm/s²  (set 0 for constant-speed belt)


# ------------------------------------------------------------------------------
# 5.  ROBOT GEOMETRY
#
#     X_MAX       — robot reach in ±X, mm
#     PICKUP_Y    — Y position (mm) where the hand picks.
#                   Must be < camera_to_robot_distance (i.e. between camera and robot).
#                   The homography maps: Y=0 → robot base, Y=150 → camera.
#                   So PICKUP_Y = 0 means pick at the robot axis; increase to give
#                   the arm more lead time.
#     PICKUP_Z    — Z depth of the pick stroke (negative = down), mm
# ------------------------------------------------------------------------------
X_MAX     = 60.0
PICKUP_Y  = 20.0    # mm — give the arm 20 mm of lead from the robot base
PICKUP_Z  = -40.0   # mm


# ------------------------------------------------------------------------------
# 6.  VISION / PATCHCORE
#
#     ROI_* fractions are proportional to frame size (0.0–1.0).
#     They define the crop window fed to PatchCore AND shown in the debug view.
#     Adjust to exclude belt edges, lighting gradients, or structural features.
# ------------------------------------------------------------------------------
ROI_TOP    = 0.2
ROI_BOTTOM = 0.9
ROI_LEFT   = 0.1
ROI_RIGHT  = 0.9

TILE_SCALE_RATIO    = 0.7    # fraction of min(crop_h, crop_w) per tile
MIN_DETECTION_AREA  = 80     # px² — minimum blob area to count
ANOMALY_PERCENTILE  = 97     # top N% of anomaly scores triggers detection
BOX_PADDING         = 10     # px — padding added around detected bounding boxes
CLUSTER_MIN_DIST    = 15     # px — merge detections within this pixel distance


# ------------------------------------------------------------------------------
# 7.  TRACKING
# ------------------------------------------------------------------------------
DUPLICATE_THRESHOLD_PIXELS = 20.0   # px — spatial dedup radius
TRACKING_TTL               = 1.0    # s  — track lifetime without updates


# ------------------------------------------------------------------------------
# 8.  MOTION PLANNER
# ------------------------------------------------------------------------------
ROBOT_DELAY        = 0.15    # s  — arm response latency estimate (adaptive)
LEARNING_RATE      = 0.0005  # correction magnitude per missed pick
DUPLICATE_DISTANCE = 15.0    # mm — suppress repeat picks at same X within 2 s


# ------------------------------------------------------------------------------
# 9.  DELTA HAND
#     MOVE_TIME and GRIP_TIME feed back into the planner's busy-time window.
#     If picks are cut short, increase these values.
# ------------------------------------------------------------------------------
MOVE_TIME  = 0.25    # s — time per IK move segment
GRIP_TIME  = 0.20    # s — gripper open/close time


# ==============================================================================
#  DO NOT EDIT BELOW THIS LINE
# ==============================================================================

NUMBER_OF_HANDS = 1   # multi-hand support reserved; do not change yet

def generate_launch_description():

    return LaunchDescription([

        # ======================================================================
        # CAMERA
        # ======================================================================
        Node(
            package='garlic_impurity_removal',
            executable='camera_node',
            parameters=[{
                'camera_index':  CAMERA_INDEX,
                'camera_width':  CAMERA_WIDTH,
                'camera_height': CAMERA_HEIGHT,
                'camera_fps':    CAMERA_FPS,
                'show_preview':  False,
                'simulate':      SIMULATE,
            }],
            output='screen'
        ),

        # ======================================================================
        # PATCHCORE  (vision / anomaly detection)
        # ======================================================================
        Node(
            package='garlic_impurity_removal',
            executable='patchcore_node',
            parameters=[{
                'tile_scale_ratio':     TILE_SCALE_RATIO,
                'min_detection_area':   MIN_DETECTION_AREA,
                'anomaly_percentile':   ANOMALY_PERCENTILE,
                'box_padding':          BOX_PADDING,
                'cluster_min_distance': CLUSTER_MIN_DIST,
                # ROI fractions — must match camera_view_node
                'roi_top':    ROI_TOP,
                'roi_bottom': ROI_BOTTOM,
                'roi_left':   ROI_LEFT,
                'roi_right':  ROI_RIGHT,
            }],
            output='screen'
        ),

        # ======================================================================
        # TRACKING  (pixel-space deduplication)
        # ======================================================================
        Node(
            package='garlic_impurity_removal',
            executable='tracking_node',
            parameters=[{
                'duplicate_threshold_pixels': DUPLICATE_THRESHOLD_PIXELS,
                'tracking_ttl':               TRACKING_TTL,
            }],
            output='screen'
        ),

        # ======================================================================
        # COORDINATE CONVERSION  (pixel -> world mm)
        # ======================================================================
        Node(
            package='garlic_impurity_removal',
            executable='coordinate_node',
            parameters=[{
                'x_max': X_MAX,
                'debug': False,
            }],
            output='screen'
        ),

        # ======================================================================
        # MOTION PLANNER
        # ======================================================================
        Node(
            package='garlic_impurity_removal',
            executable='motion_planner_node',
            parameters=[{
                'conveyor_speed':        CONVEYOR_SPEED,
                'conveyor_acceleration': CONVEYOR_ACCELERATION,
                'pickup_y':              PICKUP_Y,
                'x_max':                 X_MAX,
                'number_of_hands':       NUMBER_OF_HANDS,
                'robot_delay':           ROBOT_DELAY,
                'learning_rate':         LEARNING_RATE,
                'duplicate_distance':    DUPLICATE_DISTANCE,
            }],
            output='screen'
        ),

        # ======================================================================
        # DELTA HAND CONTROL
        # ======================================================================
        Node(
            package='garlic_impurity_removal',
            executable='delta_hand_node',
            parameters=[{
                'serial_port':     SERIAL_PORT,
                'baud_rate':       BAUD_RATE,
                'x_max':           X_MAX,
                'pickup_z':        PICKUP_Z,
                'number_of_hands': NUMBER_OF_HANDS,
                'simulate':        SIMULATE,
                'move_time':       MOVE_TIME,
                'grip_time':       GRIP_TIME,
            }],
            output='screen'
        ),

        # ======================================================================
        # DEBUG VIEW
        # ======================================================================
        Node(
            package='garlic_impurity_removal',
            executable='camera_view_node',
            parameters=[{
                'image_topic':    '/camera/image_raw',
                'display_fps':    30,
                'enable_display': True,
                'data_ttl':       1.5,
                # ROI fractions — must match patchcore_node
                'roi_top':    ROI_TOP,
                'roi_bottom': ROI_BOTTOM,
                'roi_left':   ROI_LEFT,
                'roi_right':  ROI_RIGHT,
            }],
            output='screen'
        ),
    ])