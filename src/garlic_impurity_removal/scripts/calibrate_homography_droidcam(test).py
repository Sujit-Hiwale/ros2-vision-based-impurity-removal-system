import cv2
import numpy as np
import os
import subprocess

# ===============================
# DROIDCAM STREAM URL
# ===============================

# Replace with your phone IP if it changes
DROIDCAM_URL = "http://192.168.1.74:4747/video"

# ===============================
# REAL ROBOT WORKSPACE (mm)
# ===============================

world_points = np.array([
    [-120, 80],   # bottom-left
    [120, 80],    # bottom-right
    [120, 200],   # top-right
    [-120, 200]   # top-left
], dtype=np.float32)

pixel_points = []

# ===============================
# MOUSE CLICK HANDLER
# ===============================

def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(pixel_points) < 4:
        pixel_points.append([x, y])
        print(f"Clicked: {x}, {y}")

print("\nClick 4 robot workspace corners:")
print("1️⃣ Bottom-Left")
print("2️⃣ Bottom-Right")
print("3️⃣ Top-Right")
print("4️⃣ Top-Left\n")

cv2.namedWindow("Calibration")
cv2.setMouseCallback("Calibration", click_event)

# ===============================
# CONNECT TO DROIDCAM
# ===============================

cap = cv2.VideoCapture(DROIDCAM_URL)

if not cap.isOpened():
    print("❌ Could not connect to DroidCam stream.")
    print("Check:")
    print("• Phone and PC are on same WiFi")
    print("• DroidCam app is running")
    print("• Correct IP address")
    exit()

print("📷 Connected to DroidCam")

# ===============================
# DISPLAY CAMERA + COLLECT CLICKS
# ===============================

while True:

    ret, frame = cap.read()

    if not ret:
        print("Frame not received")
        continue

    # Draw clicked points
    for p in pixel_points:
        cv2.circle(frame, tuple(p), 6, (0,255,0), -1)

    cv2.imshow("Calibration", frame)

    if len(pixel_points) == 4:
        break

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()

if len(pixel_points) != 4:
    print("Calibration cancelled.")
    exit()

pixel_points = np.array(pixel_points, dtype=np.float32)

# ===============================
# COMPUTE HOMOGRAPHY
# ===============================

H = cv2.getPerspectiveTransform(pixel_points, world_points)

print("\nHomography Matrix:\n")
print(H)

# ===============================
# FIND ROS PACKAGE INSTALL PATH
# ===============================

pkg_path = subprocess.check_output(
    ["ros2", "pkg", "prefix", "garlic_impurity_removal"]
).decode().strip()

config_path = os.path.join(
    pkg_path,
    "share",
    "garlic_impurity_removal",
    "config"
)

os.makedirs(config_path, exist_ok=True)

matrix_path = os.path.join(config_path, "homography_matrix.npy")

np.save(matrix_path, H)

print("\n✅ Calibration saved to:")
print(matrix_path)