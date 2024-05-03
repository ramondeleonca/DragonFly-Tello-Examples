from ursina import *
import cv2
import threading
import time
import numpy as np
from djitellopy import Tello
import FlyLib3.vision.apriltag as apriltag

app = Ursina(title="FlyLib3 Visualization", borderless=False, fullscreen=False)
drone_model = Entity(model=load_model("./tello.obj"), scale=(0.1, 0.1, 0.1))
camera = EditorCamera()

drone = Tello()
detector = apriltag.ApriltagDetector()

apriltag_entity = Entity(model=Plane(), texture=load_texture("./apriltag_texture.jpg"), scale=(1, 1, 1))

def rotation_matrix_to_euler(R):
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

    return np.array([x, y, z])

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
            apriltag_entity.rotation = rotation_matrix_to_euler(detection.pose_R)
            apriltag_entity.set_position(detection.pose_t, drone_model)
            for corner_index in range(4):
                cv2.circle(frame, tuple(detection.corners[corner_index].astype(int)), 5, (0, 0, 255), -1)
        
        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            drone.land()
            break
    time.sleep(0.1)

threading.Thread(target=main).start()
app.run()