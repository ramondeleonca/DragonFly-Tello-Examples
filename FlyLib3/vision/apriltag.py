from dataclasses import dataclass
import pupil_apriltags
import numpy as np
import cv2

class ApriltagFieldLayouts:
    # TODO: ADD PRINT MARGINS
    # TODO: ADD OTHER APRILTAGS
    k2024 = {
        1: {
            "t": [0.5, 0, 3],
            "r": [0, 0, 0],
        },
        2: {
            "t": [1.1694, 0, 3],
            "r": [0, 0, 0],
        },
        3: {
            "t": [0.5, 0, 2],
            "r": [0, 0, 0],
        },
        4: {
            "t": [1.1694, 0, 2],
            "r": [0, 0, 0],
        },
        5: {
            "t": [2.5674, 0, 0.5],
            "r": [0, 90, 0],
        },
    }
    kCurrent = k2024

@dataclass
class ApriltagDetectionResult:
    tag_family: str
    tag_id: int
    hamming: int
    decision_margin: float
    homography: np.ndarray
    center: np.ndarray
    corners: np.ndarray
    pose_R: np.ndarray = None
    pose_t: np.ndarray = None
    pose_err: float = None

class ApriltagDetector:
    detector: pupil_apriltags.Detector
    result: ApriltagDetectionResult = None

    def __init__(self, 
                 families: str = "tag36h11",
                 nthreads: int = 4,
                 quad_decimate: float = 4,
                 quad_sigma: float = 0.0,
                 refine_edges: int = 1,
                 decode_sharpening: float = 0.25,
                 debug: int = 0
                ) -> None:
        """FlyLib3 wrapper for pupil_apriltags.

        Apriltags can be printed from the 2024 FIRST game Apriltag user guide: https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/Apriltag_Images_and_User_Guide.pdf
        
        (KEEP IN MIND THIS IS NOT THE FIELD USED IN COMPETITION)

        :param families: Tag families, separated with a space, default: tag36h11

        :param nthreads: Number of threads, default: 4

        :param quad_decimate: Detection of quads can be done on a lower-resolution image,
            improving speed at a cost of pose accuracy and a slight decrease in detection
            rate. Decoding the binary payload is still done at full resolution, default: 4

        :param quad_sigma: What Gaussian blur should be applied to the segmented image (used
            for quad detection?)  Parameter is the standard deviation in pixels.  Very noisy
            images benefit from non-zero values (e.g. 0.8), default:  0.0

        :param refine_edges: When non-zero, the edges of the each quad are adjusted to "snap
            to" strong gradients nearby. This is useful when decimation is employed, as it
            can increase the quality of the initial quad estimate substantially. Generally
            recommended to be on (1). Very computationally inexpensive. Option is ignored
            if quad_decimate = 1, default: 1

        :param decode_sharpening: How much sharpening should be done to decoded images? This
            can help decode small tags but may or may not help in odd lighting conditions or
            low light conditions, default = 0.25

        :param debug: If 1, will save debug images. Runs very slow, default: 0

        :param searchpath: Where to look for the Apriltag 3 library, must be a list,
            default: ["src/lib", "src/lib64"]
        """
        self.detector = pupil_apriltags.Detector(families=families, nthreads=nthreads, quad_decimate=quad_decimate, quad_sigma=quad_sigma, refine_edges=refine_edges, decode_sharpening=decode_sharpening, debug=debug)
    
    def detect(self,
                 frame: np.ndarray[3, np.uint8],
                 estimate_tag_pose: bool = False,
                 tag_size_meters: float = 0.1651,
                 camera_params: tuple[float, float, float, float] = [79.56, 79.506, 640, 360],
                ) -> list[ApriltagDetectionResult]:
        """Detects AprilTags in a frame.

        :param frame: The frame to detect tags in

        :param estimate_tag_pose: Whether to estimate the tag pose, default: False

        :param tag_size_meters: The size of the tag in meters, default: 0.1651 (6.5 inches)

        :param camera_params: Camera parameters, default: [79.56, 79.506, 640, 360] (calculated from the DJI Tello camera)
        """
        # Convert the frame to grayscale if not already
        if len(frame.shape) == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        results = self.detector.detect(frame, estimate_tag_pose, camera_params, tag_size_meters)
        self.result = [
            ApriltagDetectionResult(
                tag_family=result.tag_family,
                tag_id=result.tag_id,
                hamming=result.hamming,
                decision_margin=result.decision_margin,
                homography=result.homography,
                center=result.center,
                corners=result.corners,
                pose_R=result.pose_R,
                pose_t=result.pose_t,
                pose_err=result.pose_err
            ) for result in results
        ]
        return self.result

if __name__ == "__main__":
    camera = cv2.VideoCapture(0)
    detector = ApriltagDetector()

    while True:
        ret, frame = camera.read()
        if not ret:
            break

        detection = detector.detect(frame)
        print(detection)

        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break