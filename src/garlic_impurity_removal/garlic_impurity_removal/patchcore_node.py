import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import torch
import torch.nn as nn
import torchvision.models as models
import torchvision.transforms as transforms
import numpy as np
import cv2
import faiss
from PIL import Image

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


DEVICE     = "cuda" if torch.cuda.is_available() else "cpu"
K_NEIGHBORS = 1


class PatchCoreNode(Node):

    def __init__(self):
        super().__init__('patchcore_node')

        self.bridge = CvBridge()

        # ===============================
        # PARAMETERS
        # ===============================
        self.declare_parameter('tile_scale_ratio',    0.7)
        self.declare_parameter('min_detection_area',  80)
        self.declare_parameter('anomaly_percentile',  97)
        self.declare_parameter('box_padding',         10)
        self.declare_parameter('cluster_min_distance', 15)

        self.tile_ratio   = self.get_parameter('tile_scale_ratio').value
        self.min_area     = self.get_parameter('min_detection_area').value
        self.percentile   = self.get_parameter('anomaly_percentile').value
        self.box_padding  = self.get_parameter('box_padding').value
        self.cluster_dist = self.get_parameter('cluster_min_distance').value

        # ===============================
        # LOAD FAISS INDEX
        # ===============================
        package_share = get_package_share_directory("garlic_impurity_removal")
        index_path    = os.path.join(package_share, "config", "garlic_detection.index")
        self.faiss_index = faiss.read_index(index_path)
        # ===============================
        # LOAD HOMOGRAPHY (ADD THIS)
        # ===============================
        pkg_path = get_package_share_directory('garlic_impurity_removal')
        H_path   = os.path.join(pkg_path, "config", "homography_matrix.npy")

        self.H     = np.load(H_path)
        self.H_inv = np.linalg.inv(self.H)

        # SAME world boundary (must match camera node)
        self._world_boundary = np.array([
            [-60.0, 150.0],
            [ 60.0, 150.0],
            [ 60.0,   0.0],
            [-60.0,   0.0],
        ], dtype=np.float32)

        # Convert to pixel ROI
        pts = self._world_boundary.reshape(1, -1, 2).astype(np.float32)
        pix = cv2.perspectiveTransform(pts, self.H_inv).reshape(-1, 2)

        xs = pix[:, 0]
        ys = pix[:, 1]

        self.roi_x0 = int(min(xs))
        self.roi_x1 = int(max(xs))
        self.roi_y0 = int(min(ys))
        self.roi_y1 = int(max(ys))

        # ===============================
        # FEATURE EXTRACTOR (ResNet18 backbone, layers 1-3)
        # ===============================
        resnet = models.resnet18(weights=models.ResNet18_Weights.IMAGENET1K_V1)
        resnet.eval().to(DEVICE)

        self.feature_extractor = nn.Sequential(
            resnet.conv1, resnet.bn1, resnet.relu, resnet.maxpool,
            resnet.layer1, resnet.layer2, resnet.layer3
        ).to(DEVICE)

        self.transform = transforms.Compose([
            transforms.Resize((256, 256)),
            transforms.ToTensor(),
        ])

        # ===============================
        # ROS
        # ===============================
        self.subscription = self.create_subscription(
            ROSImage, '/camera/image_raw', self.image_callback, 10
        )

        self.publisher_ = self.create_publisher(
            PoseArray, '/garlic_pixel_detections', 10
        )

        self.get_logger().info(
            f"PatchCore Node Ready | device={DEVICE} | "
            f"ROI_pixels=[({self.roi_x0},{self.roi_y0}) -> ({self.roi_x1},{self.roi_y1})]"
        )

    # ===============================
    def compute_tile_params(self, H, W):
        base      = min(H, W)
        tile_size = int(self.tile_ratio * base)
        tile_size = (tile_size // 32) * 32
        tile_size = max(128, min(tile_size, 512))
        stride    = tile_size // 2
        return tile_size, stride

    # ===============================
    def extract_features(self, tile):
        img    = Image.fromarray(cv2.cvtColor(tile, cv2.COLOR_BGR2RGB))
        tensor = self.transform(img).unsqueeze(0).to(DEVICE)

        with torch.no_grad():
            features = self.feature_extractor(tensor)

        B, C, H, W = features.shape
        features   = features.permute(0, 2, 3, 1).reshape(-1, C)
        features   = nn.functional.normalize(features, dim=1)

        return features.cpu().numpy(), H, W

    # ===============================
    def process_frame(self, frame):

        H_full, W_full    = frame.shape[:2]
        TILE_SIZE, STRIDE = self.compute_tile_params(H_full, W_full)

        anomaly_map = np.zeros((H_full, W_full))
        count_map   = np.zeros((H_full, W_full))

        for y in range(0, H_full, STRIDE):
            for x in range(0, W_full, STRIDE):

                tile = frame[y:y + TILE_SIZE, x:x + TILE_SIZE]

                if tile.shape[0] < TILE_SIZE or tile.shape[1] < TILE_SIZE:
                    continue

                features, h_feat, w_feat = self.extract_features(tile)
                distances, _             = self.faiss_index.search(features, K_NEIGHBORS)
                scores                   = distances.mean(axis=1)

                score_map = cv2.resize(scores.reshape(h_feat, w_feat), (TILE_SIZE, TILE_SIZE))

                anomaly_map[y:y + TILE_SIZE, x:x + TILE_SIZE] += score_map
                count_map[y:y + TILE_SIZE,   x:x + TILE_SIZE] += 1

        anomaly_map /= (count_map + 1e-8)
        anomaly_map  = cv2.GaussianBlur(anomaly_map, (7, 7), 0)

        threshold   = np.percentile(anomaly_map, self.percentile)
        binary_mask = (anomaly_map > threshold).astype(np.uint8)

        kernel      = np.ones((5, 5), np.uint8)
        binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN,  kernel)
        binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        detections = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            detections.append((x + w // 2, y + h // 2, area))

        return detections

    # ===============================
    def cluster_points(self, detections):
        filtered = []
        for cx, cy, area in detections:
            if all(
                np.hypot(cx - x, cy - y) > self.cluster_dist
                for x, y, _ in filtered
            ):
                filtered.append((cx, cy, area))
        return filtered

    # ===============================
    def image_callback(self, msg):

        frame  = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w   = frame.shape[:2]

        roi = frame[self.roi_y0:self.roi_y1, self.roi_x0:self.roi_x1]
        detections = self.cluster_points(self.process_frame(roi))

        msg_out             = PoseArray()
        msg_out.header      = msg.header
        msg_out.header.frame_id = "camera"

        for cx, cy, area in detections:
            pose             = Pose()
            pose.position.x = float(cx + self.roi_x0)
            pose.position.y = float(cy + self.roi_y0)
            pose.position.z  = float(area)
            msg_out.poses.append(pose)

        self.publisher_.publish(msg_out)
        self.get_logger().info(f"Published {len(detections)} detections")


def main(args=None):
    rclpy.init(args=args)
    node = PatchCoreNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()