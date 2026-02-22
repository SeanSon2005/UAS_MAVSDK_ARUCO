import cv2

from constants import RED_LOWER_1, RED_UPPER_1, RED_LOWER_2, RED_UPPER_2, RED_MIN_AREA_PX


class ColorDetector:
    def __init__(self):
        self.lower_red_1 = tuple(RED_LOWER_1)
        self.upper_red_1 = tuple(RED_UPPER_1)
        self.lower_red_2 = tuple(RED_LOWER_2)
        self.upper_red_2 = tuple(RED_UPPER_2)
        self.min_area_px = float(RED_MIN_AREA_PX)

    def detect_with_debug(self, frame):
        if frame is None:
            return None, None, None, None, None, 0.0

        blurred = cv2.GaussianBlur(frame, (9, 9), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        red_mask_1 = cv2.inRange(hsv, self.lower_red_1, self.upper_red_1)
        red_mask_2 = cv2.inRange(hsv, self.lower_red_2, self.upper_red_2)
        red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None, red_mask, None, None, 0.0

        largest_contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(largest_contour))
        if area < self.min_area_px:
            return None, None, red_mask, None, largest_contour, area

        moments = cv2.moments(largest_contour)
        if moments["m00"] == 0:
            return None, None, red_mask, None, largest_contour, area

        c_x = int(moments["m10"] / moments["m00"])
        c_y = int(moments["m01"] / moments["m00"])

        h, w = red_mask.shape[:2]
        err_x_px = float(c_x - (w / 2.0))
        err_y_px = float(c_y - (h / 2.0))

        return err_x_px, err_y_px, red_mask, (c_x, c_y), largest_contour, area
