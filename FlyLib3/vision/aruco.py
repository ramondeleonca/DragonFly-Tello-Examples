import os
import cv2
import numpy as np
from dataclasses import dataclass

@dataclass
class ArucoApriltagDetection:
    id: int
    corners: np.ndarray
    rvec: np.ndarray
    tvec: np.ndarray

class ArucoApriltagDetector:
    # Aruco dictionary and parameters
    aruco_dict: cv2.aruco.Dictionary
    aruco_params: cv2.aruco.DetectorParameters

    # Image preprocessing
    blur: int

    # Tag size in meters
    tag_size_m: float

    # Camera calibration parameters
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray

    # Preview
    preview: bool

    def __init__(
            self,
            camera_matrix: np.ndarray,
            dist_coeffs: np.ndarray,
            aruco_dict: cv2.aruco.Dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
            aruco_params: cv2.aruco.DetectorParameters = cv2.aruco.DetectorParameters(),
            blur: int = 3,
            tag_size_m: float = 0.16,
            preview: bool = False
        ) -> None:
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.aruco_dict = aruco_dict
        self.aruco_params = aruco_params
        self.tag_size_m = tag_size_m
        self.preview = preview
        self.blur = blur
    
    def detect(self, frame: np.ndarray):
        # Convert the frame to grayscale if not already
        if len(frame.shape) == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply a blur to the frame
        cv2.blur(frame, (self.blur, self.blur), frame)
        
        # Detect the markers
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        # If no markers are detected, return an empty list
        if ids is None:
            return []
        
        # Get the rotation and translation vectors
        # TODO: Switch to cv2.resolvePnP for better accuracy
        rvecs: list[np.ndarray]
        tvecs: list[np.ndarray]
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.tag_size_m, self.camera_matrix, self.dist_coeffs)

        # Create a list of detections
        detections: list[ArucoApriltagDetection] = []
        for i in range(len(ids)):
            detections.append(ArucoApriltagDetection(
                int(ids[i]),
                corners[i],
                rvecs[i],
                tvecs[i]
            ))
        
        # If preview is enabled, show the frame
        if self.preview:
            # draw markers
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # draw axis
            for i in range(len(ids)):
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)
        
            cv2.imshow('preview', frame)
        
        return detections

# Default Apriltag Detector
ApriltagDetector = ArucoApriltagDetector





# ----- TESTS -----
if __name__ == "__main__":
    import time

    tag_size = 0.16
    tag_size_center = tag_size // 2

    dirname  = os.path.dirname(__file__)
    calibration_matrix = np.load(f'{dirname}/test/assets/calibration_results/calibration_matrix.npz')['mtx']
    distortion_coefficients = np.load(f'{dirname}/test/assets/calibration_results/distortion_coefficients.npz')['dist']

    detector = ArucoApriltagDetector(calibration_matrix, distortion_coefficients, blur=3, preview=True)

    cap = cv2.VideoCapture(0)

    last_time = time.time()
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # decimate the frame
        decimation = 1
        frame = cv2.resize(frame, (frame.shape[1] // decimation, frame.shape[0] // decimation), interpolation=cv2.INTER_LINEAR)

        detections = detector.detect(frame)

        if len(detections) > 0:
            for detection in detections:
                # Draw the tag ID
                cv2.putText(frame, f"ID: {detection.id}", (250, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                # Draw the axis
                cv2.aruco.drawDetectedMarkers(frame, [detection.corners], np.array([detection.id]))

                # Draw the tag pose
                cv2.drawFrameAxes(frame, calibration_matrix, distortion_coefficients, detection.rvec, detection.tvec, 0.016)
        
        # draw framerate
        cv2.putText(frame, "FPS: {:.2f}".format(1.0 / (time.time() - last_time)), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        last_time = time.time()

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break