"""Hardware-independent geometry and filtering for steel-ball tracking."""

import math

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


def position_from_pixel(
    point,
    axis_start,
    axis_end,
    start_position_cm,
    end_position_cm,
    correction_k=0.0,
):
    """Project an image point onto the beam and convert it to centimeters.

    Args:
        correction_k: Non-linear correction coefficient (default 0 = linear).
            Positive values compress readings on the positive-cm side and
            expand readings on the negative-cm side (corrects for the camera
            being tilted toward the positive side).
            Negative values do the opposite.
            Typical range: -2.0 to 2.0.  Start with 0.6~1.0 if positive
            readings are ~20% too high.
    """
    if start_position_cm == end_position_cm:
        raise ValueError("physical calibration positions must be different")
    ratio, distance = project_onto_axis(point, axis_start, axis_end)

    if correction_k != 0.0:
        # Cubic correction term: zero at ratio=0, ratio=zero_ratio, ratio=1.
        # This preserves the calibration endpoints and the physical zero point.
        span = end_position_cm - start_position_cm
        zero_ratio = (0.0 - start_position_cm) / span
        correction = correction_k * ratio * (1.0 - ratio) * (ratio - zero_ratio)
        ratio = ratio - correction
        # Clamp ratio to [0, 1] so extreme k values can't escape the axis.
        if ratio < 0.0:
            ratio = 0.0
        elif ratio > 1.0:
            ratio = 1.0

    position_cm = start_position_cm + ratio * (
        end_position_cm - start_position_cm
    )
    return position_cm, ratio, distance


class CalibrationTable:
    """Multi-point calibration table with bilinear interpolation.

    Maps arbitrary pixel values to physical cm through a set of measured
    (pixel, cm) pairs.  Much more accurate than two-point linear mapping
    when the camera is not perfectly perpendicular to the measurement plane.
    """

    def __init__(self, points):
        """*points*: iterable of ``(pixel, cm)`` pairs.  Pixels must be
        strictly increasing.
        """
        pts = sorted(points, key=lambda p: p[0])
        self._pixels = [p[0] for p in pts]
        self._cms = [p[1] for p in pts]

        # Verify strict ordering
        for i in range(1, len(self._pixels)):
            if self._pixels[i] <= self._pixels[i - 1]:
                raise ValueError(
                    "calibration pixels must be strictly increasing, "
                    "got {} after {}".format(
                        self._pixels[i], self._pixels[i - 1]
                    )
                )
        self._count = len(self._pixels)

    def to_cm(self, pixel):
        """Convert a raw pixel coordinate to centimetres via lookup +
        linear interpolation.  Returns ``(cm, ratio, distance)`` where
        *ratio* is a normalised position [0, 1] across the full table span
        and *distance* is always 0 (no perpendicular distance concept with
        a multi-point table — the axis is implicit).
        """
        px_min = self._pixels[0]
        px_max = self._pixels[-1]

        # Clamp to table bounds
        if pixel <= px_min:
            cm = self._cms[0]
        elif pixel >= px_max:
            cm = self._cms[-1]
        else:
            # Binary search: find the first index whose pixel > given pixel
            lo, hi = 0, self._count - 1
            while lo < hi:
                mid = (lo + hi) // 2
                if self._pixels[mid] < pixel:
                    lo = mid + 1
                else:
                    hi = mid
            # lo is now the index of the first pixel >= given pixel
            i_hi = lo
            i_lo = lo - 1
            frac = (pixel - self._pixels[i_lo]) / float(
                self._pixels[i_hi] - self._pixels[i_lo]
            )
            cm = self._cms[i_lo] + frac * (
                self._cms[i_hi] - self._cms[i_lo]
            )

        # Compute a normalised ratio for compatibility with callers
        cm_min = self._cms[0]
        cm_max = self._cms[-1]
        span = cm_max - cm_min
        if span != 0:
            ratio = (cm - cm_min) / span
        else:
            ratio = 0.0

        return cm, ratio, 0.0


class AdaptiveAlphaBetaFilter:
    """Velocity-aware position filter with adaptive measurement gain."""

    def __init__(
        self,
        alpha_slow=0.20,
        alpha_fast=0.95,
        beta_slow=0.01,
        beta_fast=0.12,
        fast_error_px=10.0,
        reset_ms=250,
    ):
        self.alpha_slow = alpha_slow
        self.alpha_fast = alpha_fast
        self.beta_slow = beta_slow
        self.beta_fast = beta_fast
        self.fast_error_px = fast_error_px
        self.reset_ms = reset_ms
        self.reset()

    def reset(self):
        self.x = None
        self.y = None
        self.vx = 0.0
        self.vy = 0.0
        self.last_ms = None

    def update(self, measured_x, measured_y, now_ms):
        if self.x is None:
            self.x = float(measured_x)
            self.y = float(measured_y)
            self.last_ms = now_ms
            return self.x, self.y

        dt = (now_ms - self.last_ms) * 0.001
        dt = max(0.005, min(0.100, dt))

        predicted_x = self.x + self.vx * dt
        predicted_y = self.y + self.vy * dt
        error_x = measured_x - predicted_x
        error_y = measured_y - predicted_y
        error = math.hypot(error_x, error_y)

        motion = min(1.0, error / self.fast_error_px) ** 2
        alpha = self.alpha_slow + (self.alpha_fast - self.alpha_slow) * motion
        beta = self.beta_slow + (self.beta_fast - self.beta_slow) * motion

        self.x = predicted_x + alpha * error_x
        self.y = predicted_y + alpha * error_y
        self.vx += beta * error_x / dt
        self.vy += beta * error_y / dt
        self.last_ms = now_ms
        return self.x, self.y

    def mark_missing(self, now_ms):
        if self.last_ms is not None and now_ms - self.last_ms >= self.reset_ms:
            self.reset()


def validate_calibration(
    axis_start, axis_end, start_position_cm, end_position_cm, width, height
):
    """Validate calibration against the detector image dimensions."""
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    for name, point in (("start", axis_start), ("end", axis_end)):
        if not (0 <= point[0] < width and 0 <= point[1] < height):
            raise ValueError("{} calibration point is outside the image".format(name))
    position_from_pixel(
        axis_start,
        axis_start,
        axis_end,
        start_position_cm,
        end_position_cm,
    )
