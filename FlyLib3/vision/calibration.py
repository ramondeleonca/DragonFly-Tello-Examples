#! MODULE NOT FINISHED

import os
import cv2
import numpy as np
import glob

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
    square_size: float
    pattern_size: tuple

    def __init__(self, path: str = "./calibration_images", square_size: float = 0.012, pattern_size: tuple = (15, 10)) -> None:
        self.path = path
        self.square_size = square_size
        self.pattern_size = pattern_size

    def calibrate(self):
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        objp = np.zeros((self.pattern_size[0]*self.pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2)
        objp *= self.square_size

        objpoints = []
        imgpoints = []

        images = glob.glob(f"{self.path}/*.png")
        for fname in images:
            fname = os.path.abspath(fname)
            print(f"Processing {fname}")
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)
            print(ret, corners)
            if ret:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)
                cv2.drawChessboardCorners(img, self.pattern_size, corners2, ret)
                cv2.imshow("img", img)
                cv2.waitKey(500)
        cv2.destroyAllWindows()

        print(objpoints, imgpoints, gray.shape[::-1])
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        return mtx, dist

    def save_coefficients(self, mtx, dist):
        np.save(f"{self.path}/camera_matrix.npy", mtx)
        np.save(f"{self.path}/distortion_coefficients.npy", dist)

    def load_coefficients(self):
        mtx = np.load(f"{self.path}/camera_matrix.npy")
        dist = np.load(f"{self.path}/distortion_coefficients.npy")
        return mtx, dist

    def undistort(self, img, mtx, dist):
        h, w = img.shape[:2]
        newcameramtx, roi = cv2

if __name__ == "__main__":
    print("1. Collect calibration images")
    print("2. Calibrate camera")
    choice = int(input("Enter choice: "))

    if choice == 1:
        pc = PhotoCollector(camera_index=int(input("Enter camera index: ")))
        pc.collect()
    elif choice == 2:
        cc = CameraCalibrator()
        mtx, dist = cc.calibrate()
        cc.save_coefficients(mtx, dist)
        print("Calibration completed")