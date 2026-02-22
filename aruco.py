import cv2
import math
import numpy as np

class ArucoDetector:
    def __init__(self, tag_id=0, offset_y=0.0, offset_x=0.0, marker_length_m=0.1,
                 camera_matrix_file="camera_matrix.npy", dist_coeffs_file="dist_coeffs.npy",
                 dictionary=cv2.aruco.DICT_4X4_50):

        self.tag_id = tag_id
        self.offset_y = offset_y
        self.offset_x = offset_x
        self.marker_length_m = marker_length_m

        try:
            self.camera_matrix = np.load(camera_matrix_file)
            self.dist_coeffs = np.load(dist_coeffs_file)
        except Exception:
            self.camera_matrix = None
            self.dist_coeffs = None

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)

    def _compute_detection(self, image: np.ndarray):
        result = {
            "err_x_m": None,
            "err_y_m": None,
            "yaw_err_deg": None,
            "target_corners": None,
            "all_corners": [],
            "all_ids": None,
            "rvec": None,
            "tvec": None,
        }

        if image is None:
            return result

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        corners, ids, _ = self.detector.detectMarkers(gray)
        result["all_corners"] = corners if corners is not None else []
        result["all_ids"] = ids

        if ids is None or len(ids) == 0:
            return result

        flat_ids = ids.flatten()
        matches = np.where(flat_ids == self.tag_id)[0]
        if len(matches) == 0:
            return result

        marker_index = int(matches[0])
        target_corners = corners[marker_index]
        result["target_corners"] = target_corners

        if self.camera_matrix is None:
            return result

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            np.asarray([target_corners], dtype=np.float32),
            self.marker_length_m,
            self.camera_matrix,
            self.dist_coeffs,
        )

        if tvecs is None or len(tvecs) == 0:
            return result

        tvec = tvecs[0][0]
        result["tvec"] = tvec
        result["rvec"] = rvecs[0][0] if rvecs is not None and len(rvecs) > 0 else None
        result["err_x_m"] = float(tvec[0] - self.offset_x)
        result["err_y_m"] = float(tvec[1] - self.offset_y)

        c = target_corners[0]
        angle_rad = math.atan2(c[1][1] - c[0][1], c[1][0] - c[0][0])
        result["yaw_err_deg"] = math.degrees(angle_rad)

        return result

    def detect_and_estimate(self, image: np.ndarray):
        detection = self._compute_detection(image)
        return (
            detection["err_x_m"],
            detection["err_y_m"],
            detection["yaw_err_deg"],
            detection["target_corners"],
        )

    def detect_with_debug(self, image: np.ndarray):
        detection = self._compute_detection(image)
        return (
            detection["err_x_m"],
            detection["err_y_m"],
            detection["yaw_err_deg"],
            detection["target_corners"],
            detection["all_corners"],
            detection["all_ids"],
            detection["rvec"],
            detection["tvec"],
        )
