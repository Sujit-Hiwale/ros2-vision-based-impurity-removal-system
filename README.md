# ROS2 Robotic Impurity Detection and Removal System

Vision-guided robotic system for real-time impurity detection, localization, and automated picking using ROS2.

---

## Overview

This project presents a ROS2-based vision-guided robotic system designed to automatically detect and remove impurities in an agricultural pipeline. It integrates computer vision, coordinate transformation, and robotic actuation into a unified perception-to-action framework.

The system captures images from a camera node, detects impurities using image processing and machine learning techniques, converts detections into real-world coordinates, and actuates a robotic delta hand to perform precise picking and removal.

This project demonstrates a complete end-to-end robotic automation pipeline, bridging AI-based perception with real-world actuation.

---

## System Architecture

### Pipeline Flow

Camera Node → PatchCore Node → Tracking Node → Coordinate Transform → Motion Planner Node → Delta Hand Node

The system graph can be visualized using:

rqt_graph

---

## Features

- Real-time image acquisition using ROS2 camera node  
- Vision-based impurity detection (OpenCV / ML)  
- Pixel-to-world coordinate transformation  
- ROS2 topic-based inter-node communication  
- Robotic picking using delta hand mechanism  
- End-to-end automation (perception to action)  
- Automatic model download from Hugging Face  

---

## Project Structure

ros2-robotic-impurity-detection-and-removal/  
│  
├── src/  
│   └── garlic_impurity_removal/  
│       ├── garlic_impurity_removal/  
│       │   ├── camera_node.py  
│       │   ├── camera_view_node.py  
│       │   ├── coordinate_node.py  
│       │   ├── motion_planner_node.py  
│       │   ├── patchcore_node.py  
│       │   ├── tracking_node.py  
│       │   ├── delta_hand_node.py  
│       │   └── scan_scheduler_node.py  
│       │  
│       ├── config/  
│       │   ├── garlic_detection.index  (After download script)  
│       │   └── homography_matrix.npy  
│       │  
│       ├── launch/  
│       │   └── system_launch.py  
│       │  
│       ├── dev/  
│       │   ├── test_grip.py  
│       │   └── test_hand.py  
│       │  
│       ├── scripts/  
│       │   ├── calibrate_homography.py  
│       │   └── calibrate_homography_droidcam(test).py  
│       │  
│       ├── test/  
│       │   ├── test_copyright.py  
│       │   ├── test_flake8.py  
│       │   └── test_pep257.py  
│       │  
│       ├── package.xml  
│       └── setup.py  
│  
├── scripts/  
│   └── download_model.sh  

---

## Setup Instructions

### 1. Clone the Repository

git clone https://github.com/Sujit-Hiwale/ros2-robotic-impurity-detection-and-removal.git  
cd ros2-robotic-impurity-detection-and-removal  

---

### 2. Download Model Weights

./scripts/download_model.sh  

Model repository:  
https://huggingface.co/Sujit-Hiwale/Garlic_anomaly_detection  

---

### 3. Calibrate Camera (Important)

Before running the system, calibration must be performed to ensure accurate coordinate transformation.

Run the calibration script:

python3 src/garlic_impurity_removal/scripts/calibrate_homography.py  

This step generates or updates the homography matrix used for mapping image coordinates to real-world coordinates.

Calibration must be repeated whenever:
- The camera position is changed  
- The camera angle is adjusted  
- The workspace setup is modified  

---

### 4. Build the ROS2 Workspace

colcon build  
source install/setup.bash  

---

### 5. Run the System

ros2 launch garlic_impurity_removal system_launch.py  

---

### 6. Verify Hardware Compatibility (Optional)

Before running the full system, the development scripts inside the `dev/` directory can be used to verify hardware compatibility and basic actuation:

- test_grip.py — verifies gripping mechanism  
- test_hand.py — verifies delta hand movement  

These scripts help ensure that the robotic system is functioning correctly before integrating with the full perception pipeline.

---

### 7. Visualize ROS Graph (Optional)

To inspect node communication:

rqt_graph  

---

## Model Weights

Due to GitHub file size limitations, model files are not included in this repository.

They are hosted externally and can be downloaded using:

./scripts/download_model.sh  

Alternatively, download manually from:  
https://huggingface.co/Sujit-Hiwale/Garlic_anomaly_detection  

Place the file inside:

src/garlic_impurity_removal/config/  

---

## Dataset

The dataset used for this project is available on Kaggle:  
https://www.kaggle.com/datasets/sujitvhiwale/dry-garlic-dataset  

---

## Key Concepts Demonstrated

- Perception-to-action robotics pipeline  
- Vision-guided manipulation  
- Coordinate system transformation  
- ROS2 distributed node architecture  
- Real-time robotic control  
- Machine learning model integration in robotics systems  

---

## Use Cases

- Agricultural automation  
- Quality inspection systems  
- Industrial sorting and picking  
- Smart farming robotics  

---
