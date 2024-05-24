from FlyLib3.control.tello import Tello
from FlyLib3.vision.apriltag import ApriltagDetector
from FlyLib3.math.pid import PID
import cv2
import math
import time

TARGET_TAG_SIZE = 200

drone = Tello()
yaw_pid = PID(0.15, 0.00021, 0, setpoint=0)
height_pid = PID(0.2, 0.00021, 0, setpoint=0)
distance_pid = PID(0.2, 0.000021, 0, setpoint=TARGET_TAG_SIZE)
detector = ApriltagDetector()

last_time = time.time()
def main():
    drone.connect()
    drone.streamon()
    print(drone.get_battery())
    time.sleep(0.5)  
    drone.takeoff()
    while True:
        global last_time
        now_time = time.time()
        frame = drone.get_frame_read().frame
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(gray, estimate_tag_pose=True)
        x_center = frame.shape[1] // 2
        y_center = frame.shape[0] // 2
        # draw corners on image
        for detection in detections:
            # Get size of tag from 2 neighbor corners
            size = math.hypot(detection.corners[0][0] - detection.corners[1][0], detection.corners[0][1] - detection.corners[1][1])
            offset_x = x_center - detection.center[0]
            offset_y = y_center - detection.center[1]
            drone.send_rc_control(0, int(distance_pid(size, now_time - last_time)), -int(height_pid(offset_y, now_time - last_time)), int(yaw_pid(offset_x, now_time - last_time)))
            for j in range(4):
                cv2.circle(frame, tuple(detection.corners[j].astype(int)), 5, (0, 0, 255), -1)
        if not detections:
            drone.send_rc_control(0, 0, 0, 0)
            pass
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            drone.land()
            break
        last_time = now_time
    time.sleep(0.1)

if __name__ == "__main__":
    main()