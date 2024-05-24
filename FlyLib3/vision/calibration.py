#! MODULE NOT FINISHED BUT WORKS

import os
import cv2
import numpy as np
import glob
from typing import Any
from djitellopy import Tello

class FlyLib3TelloPhotoCollector:
    drone: Tello
    capture_key: str
    path: str

    def __init__(self, drone: Tello, capture_key = "s", path: str = "./calibration_images") -> None:
        self.drone = drone
        self.capture_key = capture_key
        self.path = path
        os.makedirs(self.path, exist_ok=True)

    def collect(self):
        try:
            self.drone.connect()
            self.drone.streamon()
        except:
            print("Error connecting to Tello drone")
            return

        i = 0
        while True:
            try:
                frame = self.drone.get_frame_read().frame
                cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.imshow("frame", frame)
            except:
                pass
            key = cv2.waitKey(1)
            if key == ord(self.capture_key):
                cv2.imwrite(f"{self.path}/image_{i}.png", frame)
                print(f"Image {i} saved")
                i += 1
            elif key == ord("q"):
                break
        
        cv2.destroyAllWindows()
class PhotoCollector:
    camera_index: int
    capture_key: str
    path: str

    def __init__(self, camera_index: int = 0, capture_key = "s", path: str = "./calibration_images") -> None:
        self.camera_index = camera_index
        self.capture_key = capture_key
        self.path = path

        os.makedirs(self.path, exist_ok=True)

    def collect(self):
        cap = cv2.VideoCapture(self.camera_index)
        i = 0
        while True:
            ret, frame = cap.read()
            cv2.imshow("frame", frame)
            key = cv2.waitKey(1)
            if key == ord(self.capture_key):
                cv2.imwrite(f"{self.path}/image_{i}.png", frame)
                print(f"Image {i} saved")
                i += 1
            elif key == ord("q"):
                break
        cap.release()
        cv2.destroyAllWindows()

class CameraCalibrator:
    path: str
    checker_size_mm: float
    checkerboard_size: tuple

    # Calibration results
    ret, mtx, dist, rvecs, tvecs = None, None, None, None, None

    def __init__(self, path: str = "./calibration_images", checker_size_mm: float = 20, checkerboard_size: tuple = (12, 10)) -> None:
        self.path = path
        self.checker_size_mm = checker_size_mm
        self.checkerboard_size = (checkerboard_size[0] - 1, checkerboard_size[1] - 1)

    def calibrate(self):
        # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(9,11,0)
        objp = np.zeros((self.checkerboard_size[0]*self.checkerboard_size[1],3), np.float32)
        objp[:,:2] = np.mgrid[0:self.checkerboard_size[0],0:self.checkerboard_size[1]].T.reshape(-1,2) * self.checker_size_mm

        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d points in real world space
        imgpoints = [] # 2d points in image plane.

        # Read images
        images = glob.glob(f'{self.path}/*.png')

        for fname in images:
            print(fname)
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, self.checkerboard_size, None)
            print(ret, corners)

            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1), criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, self.checkerboard_size, corners2, ret)

                cv2.imshow('img', img)
                cv2.waitKey(500)

        cv2.destroyAllWindows()

        # Calibrate camera
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        # Save calibration results
        self.ret = ret
        self.mtx = mtx
        self.dist = dist
        self.rvecs = rvecs
        self.tvecs = tvecs
    
    def undistort(self, img):
        h,  w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))

        # undistort
        dst = cv2.undistort(img, self.mtx, self.dist, None, newcameramtx)

        # crop the image
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
        return dst
    
    def save_calibration_matrix(self, path: str = "./calibration_results"):
        os.makedirs(path, exist_ok=True)
        np.savez(f'{path}/calibration_matrix.npz', mtx=self.mtx)

    def save_distortion_coefficients(self, path: str = "./calibration_results"):
        os.makedirs(path, exist_ok=True)
        np.savez(f'{path}/distortion_coefficients.npz', dist=self.dist)
    
    def save_rotation_vectors(self, path: str = "./calibration_results"):
        os.makedirs(path, exist_ok=True)
        np.savez(f'{path}/rotation_vectors.npz', rvecs=self.rvecs)
    
    def save_translation_vectors(self, path: str = "./calibration_results"):
        os.makedirs(path, exist_ok=True)
        np.savez(f'{path}/translation_vectors.npz', tvecs=self.tvecs)

    def save_all(self):
        self.save_calibration_matrix()
        self.save_distortion_coefficients()
        self.save_rotation_vectors()
        self.save_translation_vectors()


if __name__ == "__main__":
    print("1. Collect calibration images")
    print("2. Calibrate camera")
    choice = int(input("Enter choice: "))

    if choice == 1:
        print("1. Use camera")
        print("2. Use Tello")
        choice = int(input("Enter choice: "))
        if choice == 1:
            pc = PhotoCollector(camera_index=int(input("Enter camera index: ")))
        else:
            pc = FlyLib3TelloPhotoCollector(Tello())
        pc.collect()
    elif choice == 2:
        cc = CameraCalibrator()
        cc.calibrate()
        cc.save_all()
        print("Calibration completed")