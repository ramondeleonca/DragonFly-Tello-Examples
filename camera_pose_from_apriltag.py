import numpy as np
import cv2
import json
import threading
import time
from ursina import *
from FlyLib3.vision.aruco import ArucoApriltagDetector
from FlyLib3.math import angles

with open("calib_integrated_camera__04f2_b221__640.json", "r") as f:
    calibration = json.load(f)
    camera_matrix = np.array(calibration["camera_matrix"])
    dist_coeffs = np.array(calibration["distortion_coefficients"])

capture = cv2.VideoCapture(1)
detector = ArucoApriltagDetector(camera_matrix=camera_matrix, dist_coeffs=dist_coeffs, preview=True, tag_size_m=0.1524)
engine = Ursina(title="FlyLib3 Camera Pose Visualization", borderless=False)
# camera = EditorCamera()
# field = Entity(model=load_model("./FlyLib3/assets/TDC_BeyondTheSky.glb"), scale=(0.1, 0.1, 0.1))
camera_etty = Entity(model="cube", scale=(0.1, 0.1, 0.1), color=color.red)
apriltag_etty = Entity(model="plane", scale=(0.25, 0.5, 0.1), color=color.white)
apriltag_etty.position = (0, 0, 0)

last_time = time.time()
def loop():
    while True:
        global last_time
        now = time.time()
        dt = now - last_time
        ret, frame = capture.read()
        detections = detector.detect(frame)

        if len(detections) > 0:
            apriltag = detections[0]
            R_tag_to_cam, _ = cv2.Rodrigues(apriltag.rvec)
            R_cam_to_tag = R_tag_to_cam.T
            tvec = apriltag.tvec.reshape(3, 1)
            t_cam_to_tag = -np.dot(R_cam_to_tag, tvec)

            euler_rot_deg = np.degrees(angles.rotation_matrix_to_euler_angles(R_cam_to_tag))
            x_translation, y_translation, z_translation = t_cam_to_tag

            camera.position = (x_translation, y_translation, z_translation)
            camera.rotation_x = euler_rot_deg[0]
            camera.rotation_y = euler_rot_deg[1]
            camera.rotation_z = euler_rot_deg[2]

        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

threading.Thread(target=loop, daemon=True).start()
engine.run()