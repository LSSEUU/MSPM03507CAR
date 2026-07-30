"""Hardware-independent calibration, filtering, and quality checks."""

import math


# Existing measured installation table, converted from centimetres to mm.
# Pixel increases toward the lever's active end, which is the positive axis.
MULTIPOINT_CALIBRATION_MM = [
    (16, -120.0),
    (27, -110.0),
    (39, -100.0),
    (53, -90.0),
    (71, -80.0),
    (83, -70.0),
    (108, -60.0),
    (127, -50.0),
    (145, -40.0),
    (165, -30.0),
    (190, -20.0),
    (205, -10.0),
    (218, 0.0),
    (248, 10.0),
    (267, 20.0),
    (294, 30.0),
    (318, 40.0),
    (331, 50.0),
    (357, 60.0),
    (383, 70.0),
    (403, 80.0),
    (422, 90.0),
    (436, 100.0),
    (454, 110.0),
    (465, 120.0),
]

# Minimal H3 calibration option using only -50, O, and +50 mm.
THREE_POINT_CALIBRATION_MM = [
    (127, -50.0),
    (218, 0.0),
    (331, 50.0),
]


def project_onto_axis(point, axis_start, axis_end):
    """Return normalized position and perpendicular distance to an image axis."""
    px, py = point
    x0, y0 = axis_start
    x1, y1 = axis_end
    dx = x1 - x0
    dy = y1 - y0
    length_squared = dx * dx + dy * dy
    if length_squared <= 0:
        raise ValueError("calibration endpoints must be different")

    relative_x = px - x0
    relative_y = py - y0
    ratio = (relative_x * dx + relative_y * dy) / float(length_squared)
    projected_x = x0 + ratio * dx
    projected_y = y0 + ratio * dy
    distance = math.hypot(px - projected_x, py - projected_y)
    return ratio, distance


def axis_point(ratio, axis_start, axis_end):
    """Return the image-space point at *ratio* along an axis."""
    x0, y0 = axis_start
    x1, y1 = axis_end
    return (
        x0 + ratio * (x1 - x0),
        y0 + ratio * (y1 - y0),
    )


class CalibrationTable:
    """Piecewise-linear pixel-to-millimetre calibration."""

    def __init__(self, points):
        points = list(points)
        if len(points) < 2:
            raise ValueError("calibration requires at least two points")

        self._pixels = [float(point[0]) for point in points]
        self._positions_mm = [float(point[1]) for point in points]
        position_direction = self._positions_mm[1] - self._positions_mm[0]
        if position_direction == 0.0:
            raise ValueError("calibration positions must be strictly monotonic")
        for index in range(1, len(points)):
            if self._pixels[index] <= self._pixels[index - 1]:
                raise ValueError("calibration pixels must be strictly increasing")
            position_step = self._positions_mm[index] - self._positions_mm[index - 1]
            if position_step * position_direction <= 0.0:
                raise ValueError("calibration positions must be strictly monotonic")
        self._count = len(points)

    def to_mm(self, pixel):
        """Interpolate in-range and linearly extrapolate from the nearest segment."""
        pixel = float(pixel)
        if not math.isfinite(pixel):
            raise ValueError("pixel must be finite")

        if pixel <= self._pixels[0]:
            low = 0
        elif pixel >= self._pixels[-1]:
            low = self._count - 2
        else:
            lo = 0
            hi = self._count - 1
            while lo < hi:
                mid = (lo + hi) // 2
                if self._pixels[mid] < pixel:
                    lo = mid + 1
                else:
                    hi = mid
            low = lo - 1

        high = low + 1
        fraction = (pixel - self._pixels[low]) / (
            self._pixels[high] - self._pixels[low]
        )
        return self._positions_mm[low] + fraction * (
            self._positions_mm[high] - self._positions_mm[low]
        )


class AdaptiveAlphaBetaFilter:
    """One-dimensional alpha-beta filter whose state is mm and mm/s."""

    def __init__(
        self,
        alpha_slow=0.20,
        alpha_fast=0.95,
        beta_slow=0.01,
        beta_fast=0.12,
        fast_error_mm=4.0,
        reset_ms=250,
    ):
        self.alpha_slow = alpha_slow
        self.alpha_fast = alpha_fast
        self.beta_slow = beta_slow
        self.beta_fast = beta_fast
        self.fast_error_mm = fast_error_mm
        self.reset_ms = reset_ms
        self.reset()

    def reset(self):
        self.x_mm = None
        self.vx_mm_s = 0.0
        self.last_ms = None

    def update(self, measured_mm, now_ms):
        measured_mm = float(measured_mm)
        if not math.isfinite(measured_mm):
            raise ValueError("measured position must be finite")

        if self.x_mm is None:
            self.x_mm = measured_mm
            self.vx_mm_s = 0.0
            self.last_ms = now_ms
            return self.x_mm, self.vx_mm_s

        dt = (now_ms - self.last_ms) * 0.001
        dt = max(0.005, min(0.100, dt))
        predicted_mm = self.x_mm + self.vx_mm_s * dt
        error_mm = measured_mm - predicted_mm
        motion = min(1.0, abs(error_mm) / self.fast_error_mm) ** 2
        alpha = self.alpha_slow + (self.alpha_fast - self.alpha_slow) * motion
        beta = self.beta_slow + (self.beta_fast - self.beta_slow) * motion

        self.x_mm = predicted_mm + alpha * error_mm
        self.vx_mm_s += beta * error_mm / dt
        self.last_ms = now_ms
        return self.x_mm, self.vx_mm_s

    def mark_missing(self, now_ms):
        if self.last_ms is not None and now_ms - self.last_ms >= self.reset_ms:
            self.reset()


def measurement_is_valid(
    score,
    box_width,
    box_height,
    center,
    axis_start,
    axis_end,
    position_mm,
    min_confidence=0.50,
    max_axis_error_px=20.0,
    min_box_size_px=6.0,
    max_box_size_px=64.0,
    min_aspect_ratio=0.50,
    max_aspect_ratio=2.0,
    min_position_mm=-115.0,
    max_position_mm=115.0,
):
    """Return ``(valid, axis_error_px)`` for one detector candidate."""
    values = (
        score,
        box_width,
        box_height,
        center[0],
        center[1],
        position_mm,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return False, float("inf")

    _, axis_error_px = project_onto_axis(center, axis_start, axis_end)
    if score < min_confidence:
        return False, axis_error_px
    if not min_box_size_px <= box_width <= max_box_size_px:
        return False, axis_error_px
    if not min_box_size_px <= box_height <= max_box_size_px:
        return False, axis_error_px

    aspect_ratio = box_width / float(box_height)
    if not min_aspect_ratio <= aspect_ratio <= max_aspect_ratio:
        return False, axis_error_px
    if axis_error_px > max_axis_error_px:
        return False, axis_error_px
    if not min_position_mm <= position_mm <= max_position_mm:
        return False, axis_error_px
    return True, axis_error_px
