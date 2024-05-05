from ursina import *
import cv2
import threading
import time
import numpy as np
from FlyLib3.control.unofficial_tello import Tello
import FlyLib3.vision.apriltag as apriltag

app = Ursina(title="FlyLib3 Visualization", borderless=False, fullscreen=False)
drone_model = Entity(model=load_model("./tello.obj"), scale=(0.1, 0.1, 0.1))
camera = EditorCamera()

drone = Tello()
detector = apriltag.ApriltagDetector()

apriltag_entity = Entity(model=Plane(), texture=load_texture("./apriltag_texture.jpg"), scale=(1, 1, 1))

def rotation_matrix_to_euler(R, mult: int = 1):
    sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    
    singular = sy < 1e-6

    if not singular :
        x = np.arctan2(R[2,1] , R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else :
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0

    return np.array([x * mult, y * mult, z * mult])

def main():
    drone.connect()
    drone.streamon()
    print(drone.get_battery())
    # drone.takeoff()
    while True:
        frame = drone.get_frame_read().frame
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        detections = detector.detect(frame, estimate_tag_pose=True)

        for detection in detections:
            print(detection)
            apriltag_entity.rotation = rotation_matrix_to_euler(detection.pose_R, mult=1000)
            apriltag_entity.set_position(detection.pose_t * 100, drone_model)
            for corner_index in range(4):
                cv2.circle(frame, tuple(detection.corners[corner_index].astype(int)), 5, (0, 0, 255), -1)
        
        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            drone.land()
            break
    time.sleep(0.1)

threading.Thread(target=main).start()
app.run()


"""
ApriltagDetectionResult(
    tag_family=b'tag36h11',
    tag_id=2,
    hamming=0,
    decision_margin=67.46334838867188,
    homography=array([
        [ 6.83473409e+01, -3.65816535e+01,  6.17329226e+02],
        [ 9.74284761e+00,  4.77052023e+01,  6.02641750e+02],
        [-1.07875295e-02, -4.17443137e-02,  1.00000000e+00]
    ]),
    center=array([617.32922557, 602.64174984]),
    corners=array([
        [528.76922607, 661.06866455],
        [685.08361816, 696.68811035],
        [700.57080078, 547.72363281],
        [556.33807373, 517.98309326]
    ]),
    pose_R=array([
        [ 9.89154098e-01, -1.45109895e-01,  2.27439626e-02],
        [ 1.45136290e-01,  9.89411549e-01,  4.94624414e-04],
        [-2.25749142e-02,  2.81171460e-03,  9.99741200e-01]
    ]),
    pose_t=array([
        [-0.02618148],
        [ 0.26863184],
        [ 0.08692467]
    ]), pose_err=6.554092223571913e-05)
"""