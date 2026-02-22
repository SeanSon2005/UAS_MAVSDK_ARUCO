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
        except Exception as e:
            self.camera_matrix = None
            self.dist_coeffs = None

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)

    def detect_and_estimate(self, image: np.ndarray):
        if image is None or self.camera_matrix is None:
            return None, None, None, None

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            return None, None, None, None

        flat_ids = ids.flatten()
        matches = np.where(flat_ids == self.tag_id)[0]
        if len(matches) == 0:
            return None, None, None, None
            
        marker_index = int(matches[0])
        target_corners = corners[marker_index]

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            np.asarray([target_corners], dtype=np.float32),
            self.marker_length_m, self.camera_matrix, self.dist_coeffs
        )

        if tvecs is not None and len(tvecs) > 0:
            tvec = tvecs[0][0] 
            err_x_m = float(tvec[0] - self.offset_x) 
            err_y_m = float(tvec[1] - self.offset_y) 

            c = target_corners[0]
            angle_rad = math.atan2(c[1][1] - c[0][1], c[1][0] - c[0][0])
            yaw_err_deg = math.degrees(angle_rad)

            return err_x_m, err_y_m, yaw_err_deg, target_corners
            
        return None, None, None, target_corners