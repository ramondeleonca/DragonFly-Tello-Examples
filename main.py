import cv2
import time
from simple_pid import PID
from djitellopy import Tello
from pupil_apriltags import Detector

drone = Tello()
pid = PID(0.23, 0.00021, 0, setpoint=0)
detector = Detector(nthreads=4, quad_decimate=4)

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
        detection = detector.detect(gray)
        frame_center = frame.shape[1] // 2
        # draw corners on image
        for i, detection in enumerate(detection):
            offset = frame_center - detection.center[0]
            print(offset)
            drone.send_rc_control(0, 0, 0, int(pid(offset, now_time - last_time)))
            for j in range(4):
                cv2.circle(frame, tuple(detection.corners[j].astype(int)), 5, (0, 0, 255), -1)
        if not detection:
            drone.send_rc_control(0, 0, 0, 0)
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            drone.land()
            break
        last_time = now_time
    time.sleep(0)

if __name__ == "__main__":
    main()