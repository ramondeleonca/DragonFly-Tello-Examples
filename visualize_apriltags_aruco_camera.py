import cv2
import numpy as np
import threading
import math
from ursina import *
from FlyLib3.vision.aruco import ArucoApriltagDetector

# Load the camera calibration parameters
camera_matrix = np.load("./FlyLib3/vision/test/assets/calibration_results/calibration_matrix.npz")["mtx"]
distortion_coefficients = np.load("./FlyLib3/vision/test/assets/calibration_results/distortion_coefficients.npz")["dist"]

# Create an ArucoApriltagDetector object
detector = ArucoApriltagDetector(camera_matrix, distortion_coefficients, blur=3, preview=True)

# Create a window
app = Ursina(borderless=False, fullscreen=False, title="FlyLib3 Visualization")

# Create an Apriltag entity
apriltag_entity = Entity(model=Plane(), texture=load_texture("./FlyLib3/assets/apriltag_texture.jpg"), scale=(1, 1, 1))

# Create a camera
camera = EditorCamera()

# Create the main loop
def update_loop():
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Detect ArUco markers in the frame
        detections = detector.detect(frame)

        for detection in detections:
            print(detection.rvec, detection.tvec)

            apriltag_entity.rotation_x = math.degrees(detection.rvec[0][0])
            apriltag_entity.rotation_y = math.degrees(detection.rvec[0][1])
            apriltag_entity.rotation_z = math.degrees(detection.rvec[0][2])

            apriltag_entity.position = detection.tvec[0] * 10

            cv2.aruco.drawDetectedMarkers(frame, [detection.corners], np.array([detection.id]))

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(0.1)

# Create a thread to run the main loop
threading.Thread(target=update_loop, daemon=True).start()

# Run the app
app.run()