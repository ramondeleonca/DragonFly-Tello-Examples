import cv2
import time
from FlyLib3.control.tello import Tello
import FlyLib3.vision.aruco as aruco

drone = Tello()
detector = aruco.ApriltagDetector()

if __name__ == "__main__":
    drone.connect()
    drone.streamon()
    print(drone.get_battery())
    # drone.takeoff()
    while True:
        frame = drone.get_frame_read().frame
        detections = detector.detect(frame, estimate_tag_pose=True)


        for detection in detections:
            print(detection)
            for corner_index in range(4):
                cv2.circle(frame, tuple(detection.corners[corner_index].astype(int)), 5, (0, 0, 255), -1)
        
        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            drone.land()
            break
    time.sleep(0.1)