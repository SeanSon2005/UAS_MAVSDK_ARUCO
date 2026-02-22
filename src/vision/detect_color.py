from __future__ import annotations

import cv2


class ColorDetector:
    def __init__(
        self,
        red_lower_1: tuple[int, int, int],
        red_upper_1: tuple[int, int, int],
        red_lower_2: tuple[int, int, int],
        red_upper_2: tuple[int, int, int],
        min_area_px: float,
        frame_color_order: str,
    ):
        self.lower_red_1 = tuple(red_lower_1)
        self.upper_red_1 = tuple(red_upper_1)
        self.lower_red_2 = tuple(red_lower_2)
        self.upper_red_2 = tuple(red_upper_2)
        self.min_area_px = float(min_area_px)
        self.frame_color_order = str(frame_color_order).upper()

    def detect_with_debug(self, frame):
        if frame is None:
            return None, None, None, None, None, 0.0

        blurred = cv2.GaussianBlur(frame, (9, 9), 0)
        hsv_code = cv2.COLOR_RGB2HSV if self.frame_color_order == "RGB" else cv2.COLOR_BGR2HSV
        hsv = cv2.cvtColor(blurred, hsv_code)

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
