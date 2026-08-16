"""Calibrate the white bar, locate the ball, and show its millimetre position."""

import gc
import math
import os
import time

import image

from machine import FPIOA, UART
from media.display import Display
from media.media import MediaManager
from media.sensor import Sensor


# ================================ User settings ==============================

IMAGE_W = 640
IMAGE_H = 480
SENSOR_FPS = 90

# Crop the camera frame before any image processing.




#ROI_TOP_LEFT = (24, 155)
#ROI_BOTTOM_RIGHT = (625, 217)
# Enter only the top-left and bottom-right points in full-frame coordinates.
ROI_TOP_LEFT = (24, 146)
ROI_BOTTOM_RIGHT = (625, 211)

CAMERA_HMIRROR = True
CAMERA_VFLIP = True

# The bar is white; the background and steel ball are black after thresholding.
#(19, 99, -76, 4, 11, 51)
#LAB_THRESHOLD = (19, 99, -76, 4, 11, 51)
#LAB_THRESHOLD = (19, 99, -76, 2, 1, 51)
LAB_THRESHOLD = (15, 90, -66, -25, -28, 39)
# Clamp both points to the full camera frame. The bottom-right point is the
# exclusive crop boundary, matching (x, y, width, height) ROI conventions.
ROI_LEFT = max(0, min(IMAGE_W - 1, int(ROI_TOP_LEFT[0])))
ROI_TOP = max(0, min(IMAGE_H - 1, int(ROI_TOP_LEFT[1])))
ROI_RIGHT = max(
    ROI_LEFT + 1, min(IMAGE_W, int(ROI_BOTTOM_RIGHT[0])))
ROI_BOTTOM = max(
    ROI_TOP + 1, min(IMAGE_H, int(ROI_BOTTOM_RIGHT[1])))
PROCESS_ROI = (
    ROI_LEFT,
    ROI_TOP,
    ROI_RIGHT - ROI_LEFT,
    ROI_BOTTOM - ROI_TOP,
)
PROCESS_W = PROCESS_ROI[2]
PROCESS_H = PROCESS_ROI[3]
DETECT_ROI = (0, 0, PROCESS_W, PROCESS_H)

MIN_BLOB_PIXELS = 800
MIN_BLOB_AREA = 1200
BLOB_MERGE = True
BLOB_MERGE_MARGIN = 3

# White bar shape filter.
BAR_MIN_LONG_SIDE_PX = 200
BAR_MAX_SHORT_SIDE_PX = 200
BAR_MIN_ASPECT_RATIO = 3.0
BAR_MAX_ASPECT_RATIO = 30.0
BAR_MIN_FILL_RATIO = 0.25

# Rotated-rectangle temporal stabilization.
RECT_SMOOTH_ALPHA = 0.25
RECT_MAX_CORNER_JUMP_PX = 50.0
RECT_HOLD_FRAMES = 10

# Steel-ball detection in the bar binary image. The bar is white and the
# steel ball is black, so this threshold selects black pixels only.
BALL_BLACK_THRESHOLD = (0, 20, -10, 10, -10, 10)
BALL_MIN_PIXELS = 48
BALL_MIN_AREA = 64
BALL_BLOB_MERGE = False
BALL_BLOB_MERGE_MARGIN = 0
BALL_MIN_DIAMETER_PX = 8
BALL_MAX_DIAMETER_PX = 80

# Expand black regions by this many pixels before finding the ball.
# The binary image uses white for the bar and black for the ball, so eroding
# the white foreground is equivalent to dilating the black ball.
# 0=disabled, 1=one 3x3 erosion (recommended), do not normally use more than 1.
BALL_BLACK_DILATE_SIZE = 0

# A fast ball is motion-blurred into a short horizontal streak. Keep a small
# short-side limit, then use circularity for initial acquisition and temporal
# continuity for motion tracking.
BALL_MIN_SHORT_SIDE_PX = 3
BALL_MIN_ASPECT_RATIO = 0.50
BALL_MAX_ASPECT_RATIO = 6.00
BALL_MIN_FILL_RATIO = 0.12
# Circle score combines width/height similarity with how closely the blob
# density matches a filled circle (pi/4 ~= 0.785). Raise the minimum to reject
# more irregular black regions during initial acquisition.
BALL_EDGE_MARGIN_PX = 3

BALL_IDEAL_CIRCLE_FILL_RATIO = 0.785
BALL_MIN_CIRCLE_SCORE = 0.45
# Once the ball has been acquired, accept a motion-blurred streak near its
# previous position with a lower circle score.
BALL_MOTION_MIN_CIRCLE_SCORE = 0.12
BALL_TRACK_SEARCH_DISTANCE_PX = 160.0
BALL_REFERENCE_HOLD_FRAMES = 2
# Ball centre must stay in the middle band of the red bar rectangle.
# This is the maximum distance from the bar centreline as a fraction of the
# bar's short side: 0.25 keeps the central 50% of the bar width.
BALL_MAX_AXIS_DISTANCE_RATIO = 0.25

# Actual distance between the left and right ends of the detected bar.
BAR_LENGTH_MM = 250.0

# UART2 output: K230 IO5 (TX) -> controller RX, IO6 (RX) is optional.
UART_TX_PIN = 5
UART_RX_PIN = 6
UART_BAUDRATE = 115200
UART_SEND_INTERVAL_FRAMES = 1
UART_LOG_INTERVAL_FRAMES = 120
# After this startup delay, keep transmitting the most recent valid position
# when the ball is temporarily missing from the current camera frame.
UART_LOSS_HOLD_DELAY_MS = 5000

# Fixed 10-byte binary frame:
# AA 55 SEQ POS10_L POS10_H VEL_L VEL_H CONF FLAGS CRC8
# POS10 is distance from the left end in 0.1 mm units. For example, 123.4 mm
# is sent as int16 value 1234. Invalid POS10 is -32768.
# FLAGS bit0=position valid, bit1=250 mm calibration active.
# FLAGS bit2=the position is being held after a detection dropout.
UART_HEADER_0 = 0xAA
UART_HEADER_1 = 0x55
UART_INVALID_POSITION_X10 = -32768
UART_FLAG_POSITION_VALID = 0x01
UART_FLAG_CALIBRATED = 0x02
UART_FLAG_POSITION_HELD = 0x04

# Steel-ball centre-point stabilization.
# 1.0 means no smoothing and the fastest response; lower values reduce jitter
# but add visible lag.
BALL_SMOOTH_ALPHA = 1.0
# A larger limit permits fast ball motion without rejecting the new position.
BALL_MAX_CENTER_JUMP_PX = 1000.0
# Only keep the old point briefly when the ball is temporarily not detected.
BALL_HOLD_FRAMES = 0

# Three-panel IDE preview settings.
# Detection runs only on the cropped PROCESS_W x PROCESS_H image.
PREVIEW_PANEL_W = 320
PREVIEW_PANEL_H = 240
DISPLAY_FPS = 60
DISPLAY_QUALITY = 60
TITLE_HEIGHT = 22
TITLE_FONT_SIZE = 16

# Overlay appearance.
BAR_ROTATED_RECT_COLOR = (255, 0, 0)
BAR_CORNER_COLOR = (255, 0, 255)
BALL_POINT_COLOR = (0, 255, 255)
DISTANCE_TEXT_COLOR = (255, 0, 0)
TEXT_COLOR = (255, 255, 255)
SEPARATOR_COLOR = (80, 80, 80)
BAR_LINE_THICKNESS = 4
BAR_CORNER_RADIUS = 6
BAR_CORNER_THICKNESS = 4
BALL_POINT_RADIUS = 6
DISTANCE_TEXT_X = 8
DISTANCE_TEXT_Y = 8
DISTANCE_FONT_SIZE = 48

# Diagnostic serial output. Periodic logs describe the bar geometry, every
# rejected ball candidate, the final millimetre result, FPS, and free memory.
DIAGNOSTIC_LOGS = False
LOG_INTERVAL_FRAMES = 15
LOG_ON_STATE_CHANGE = False
# Lightweight performance report while full diagnostics are disabled.
FPS_LOG_INTERVAL_FRAMES = 120
GC_INTERVAL_FRAMES = 60

# =============================================================================

PANEL_W = PREVIEW_PANEL_W
PANEL_H = PREVIEW_PANEL_H
DISPLAY_W = PANEL_W * 3
DISPLAY_H = PANEL_H + TITLE_HEIGHT
PREVIEW_SCALE = min(
    PANEL_W / float(PROCESS_W),
    PANEL_H / float(PROCESS_H),
)
PREVIEW_DRAW_W = int(round(PROCESS_W * PREVIEW_SCALE))
PREVIEW_DRAW_H = int(round(PROCESS_H * PREVIEW_SCALE))
PREVIEW_X_OFFSET = (PANEL_W - PREVIEW_DRAW_W) // 2
PREVIEW_Y_OFFSET = (PANEL_H - PREVIEW_DRAW_H) // 2


def init_uart2():
    """Configure IO5/IO6 and open UART2 at 115200 8N1."""
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, fpioa.UART2_TXD)
    fpioa.set_function(UART_RX_PIN, fpioa.UART2_RXD)
    return UART(
        UART.UART2,
        baudrate=UART_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )


def uart_crc8_atm(data):
    """CRC-8/ATM: polynomial 0x07, initial value 0x00."""
    crc = 0
    for value in data:
        crc ^= int(value) & 0xFF
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def append_uart_int16_le(buffer, value):
    value = int(round(value))
    value = max(-32768, min(32767, value))
    if value < 0:
        value += 65536
    buffer.append(value & 0xFF)
    buffer.append((value >> 8) & 0xFF)


def build_uart_distance_frame(sequence, distance_mm, held=False):
    """Build one position frame; distance_mm=None means invalid detection."""
    valid = distance_mm is not None
    position_x10 = (
        int(round(distance_mm * 10.0))
        if valid else UART_INVALID_POSITION_X10
    )
    confidence = 255 if valid else 0
    flags = UART_FLAG_CALIBRATED
    if valid:
        flags |= UART_FLAG_POSITION_VALID
    if valid and held:
        flags |= UART_FLAG_POSITION_HELD

    frame = bytearray()
    frame.append(UART_HEADER_0)
    frame.append(UART_HEADER_1)
    frame.append(int(sequence) & 0xFF)
    append_uart_int16_le(frame, position_x10)
    append_uart_int16_le(frame, 0)
    frame.append(confidence)
    frame.append(flags)
    frame.append(uart_crc8_atm(frame))
    return bytes(frame)


def select_bar_blob(blobs):
    """Select the largest long white blob that can represent the bar."""
    candidates = []
    for blob in blobs:
        rect = blob.rect()
        width = int(rect[2])
        height = int(rect[3])
        if width <= 0 or height <= 0:
            continue

        long_side = max(width, height)
        short_side = min(width, height)
        aspect_ratio = long_side / float(short_side)
        fill_ratio = blob.pixels() / float(width * height)

        if long_side < BAR_MIN_LONG_SIDE_PX:
            continue
        if short_side > BAR_MAX_SHORT_SIDE_PX:
            continue
        if aspect_ratio < BAR_MIN_ASPECT_RATIO:
            continue
        if aspect_ratio > BAR_MAX_ASPECT_RATIO:
            continue
        if fill_ratio < BAR_MIN_FILL_RATIO:
            continue

        score = blob.pixels() + fill_ratio
        candidates.append((score, blob, aspect_ratio, fill_ratio))

    if not candidates:
        return None, 0.0, 0.0

    candidates.sort(key=lambda item: item[0], reverse=True)
    _, blob, aspect_ratio, fill_ratio = candidates[0]
    return blob, aspect_ratio, fill_ratio


def distance(point_a, point_b):
    dx = float(point_a[0] - point_b[0])
    dy = float(point_a[1] - point_b[1])
    return math.sqrt(dx * dx + dy * dy)


def rectangle_geometry(corners):
    edge_lengths = [
        distance(corners[index], corners[(index + 1) & 3])
        for index in range(4)
    ]
    long_side = max(edge_lengths)
    short_side = min(edge_lengths)
    aspect_ratio = long_side / max(1.0, short_side)
    area = long_side * short_side

    first_short_edge = min(
        range(4), key=lambda index: edge_lengths[index])
    short_edge_indices = (first_short_edge, (first_short_edge + 2) & 3)
    axis_points = []
    for index in short_edge_indices:
        point_a = corners[index]
        point_b = corners[(index + 1) & 3]
        axis_points.append((
            int(round((point_a[0] + point_b[0]) * 0.5)),
            int(round((point_a[1] + point_b[1]) * 0.5)),
        ))

    center = (
        int(round(sum(point[0] for point in corners) / 4.0)),
        int(round(sum(point[1] for point in corners) / 4.0)),
    )
    return {
        "corners": corners,
        "long_side": long_side,
        "short_side": short_side,
        "aspect_ratio": aspect_ratio,
        "area": area,
        "axis_points": axis_points,
        "center": center,
    }


def draw_rotated_rectangle(img, corners, color, thickness):
    for index in range(4):
        point_a = corners[index]
        point_b = corners[(index + 1) & 3]
        img.draw_line(
            int(point_a[0]),
            int(point_a[1]),
            int(point_b[0]),
            int(point_b[1]),
            color=color,
            thickness=thickness,
        )


def corners_bounding_roi(corners):
    left = max(0, min(int(point[0]) for point in corners))
    top = max(0, min(int(point[1]) for point in corners))
    right = min(PROCESS_W - 1, max(int(point[0]) for point in corners))
    bottom = min(PROCESS_H - 1, max(int(point[1]) for point in corners))
    width = right - left + 1
    height = bottom - top + 1
    if width <= 1 or height <= 1:
        return None
    return (left, top, width, height)


def point_inside_convex_quad(point, corners):
    """Return True when point lies inside a cyclic convex quadrilateral."""
    has_positive = False
    has_negative = False
    px, py = point
    for index in range(4):
        point_a = corners[index]
        point_b = corners[(index + 1) & 3]
        cross = (
            (point_b[0] - point_a[0]) * (py - point_a[1]) -
            (point_b[1] - point_a[1]) * (px - point_a[0])
        )
        if cross > 0:
            has_positive = True
        elif cross < 0:
            has_negative = True
        if has_positive and has_negative:
            return False
    return True


def point_to_line_distance(point, line_start, line_end):
    dx = float(line_end[0] - line_start[0])
    dy = float(line_end[1] - line_start[1])
    length = math.sqrt(dx * dx + dy * dy)
    if length < 0.000001:
        return 1000000.0
    numerator = abs(
        dy * point[0] - dx * point[1] +
        line_end[0] * line_start[1] -
        line_end[1] * line_start[0]
    )
    return numerator / length


def ball_distance_from_left_mm(ball_point, axis_point_a, axis_point_b):
    """Project the ball centre onto the bar axis and map it to 0..250 mm."""
    if axis_point_a[0] <= axis_point_b[0]:
        left_point = axis_point_a
        right_point = axis_point_b
    else:
        left_point = axis_point_b
        right_point = axis_point_a

    axis_x = float(right_point[0] - left_point[0])
    axis_y = float(right_point[1] - left_point[1])
    axis_length_squared = axis_x * axis_x + axis_y * axis_y
    if axis_length_squared < 1.0:
        return None

    ball_x = float(ball_point[0] - left_point[0])
    ball_y = float(ball_point[1] - left_point[1])
    position_ratio = (
        ball_x * axis_x + ball_y * axis_y
    ) / axis_length_squared
    position_ratio = max(0.0, min(1.0, position_ratio))
    return position_ratio * BAR_LENGTH_MM


def select_ball_blob(
    blobs,
    corners,
    rectangle_info,
    reference_point=None,
):
    """Select the ball blob and return rejection statistics for diagnostics."""
    axis_start = rectangle_info["axis_points"][0]
    axis_end = rectangle_info["axis_points"][1]
    max_axis_distance = (
        BALL_MAX_AXIS_DISTANCE_RATIO * rectangle_info["short_side"])
    candidates = []
    diagnostics = None
    if DIAGNOSTIC_LOGS:
        diagnostics = {
            "total": len(blobs),
            "outside": 0,
            "edge": 0,
            "size": 0,
            "thin": 0,
            "aspect": 0,
            "fill": 0,
            "circle": 0,
            "axis": 0,
            "accepted": 0,
            "selected_rect": None,
            "selected_pixels": 0,
            "selected_aspect": 0.0,
            "selected_fill": 0.0,
            "selected_circle": 0.0,
            "selected_track_dist": -1.0,
            "selected_axis_dist": 0.0,
        }

    for blob in blobs:
        rect = blob.rect()
        width = int(rect[2])
        height = int(rect[3])
        if width <= 0 or height <= 0:
            continue

        center = (int(blob.cx()), int(blob.cy()))
        if not point_inside_convex_quad(center, corners):
            if diagnostics is not None:
                diagnostics["outside"] += 1
            continue

        # Reject points too close to any edge of the bar rectangle.
        too_close = False
        for i in range(4):
            if point_to_line_distance(
                center, corners[i], corners[(i + 1) & 3]
            ) < BALL_EDGE_MARGIN_PX:
                too_close = True
                break
        if too_close:
            if diagnostics is not None:
                diagnostics["edge"] += 1
            continue

        diameter = max(width, height)
        if diameter < BALL_MIN_DIAMETER_PX:
            if diagnostics is not None:
                diagnostics["size"] += 1
            continue
        if diameter > BALL_MAX_DIAMETER_PX:
            if diagnostics is not None:
                diagnostics["size"] += 1
            continue
        if min(width, height) < BALL_MIN_SHORT_SIDE_PX:
            if diagnostics is not None:
                diagnostics["thin"] += 1
            continue

        aspect_ratio = width / float(height)
        if aspect_ratio < BALL_MIN_ASPECT_RATIO:
            if diagnostics is not None:
                diagnostics["aspect"] += 1
            continue
        if aspect_ratio > BALL_MAX_ASPECT_RATIO:
            if diagnostics is not None:
                diagnostics["aspect"] += 1
            continue

        fill_ratio = blob.pixels() / float(width * height)
        if fill_ratio < BALL_MIN_FILL_RATIO:
            if diagnostics is not None:
                diagnostics["fill"] += 1
            continue

        # A filled circle has width ~= height and covers pi/4 of its bounding
        # square. Combining both properties rejects large pipe-end fragments
        # and reflections without running the slower Hough circle detector.
        axis_roundness = min(width, height) / float(max(width, height))
        if fill_ratio <= 0.0:
            fill_similarity = 0.0
        else:
            fill_similarity = min(
                fill_ratio / BALL_IDEAL_CIRCLE_FILL_RATIO,
                BALL_IDEAL_CIRCLE_FILL_RATIO / fill_ratio,
            )
        circle_score = axis_roundness * fill_similarity
        if reference_point is None:
            tracking_distance = None
            minimum_circle_score = BALL_MIN_CIRCLE_SCORE
        else:
            tracking_distance = distance(center, reference_point)
            if tracking_distance <= BALL_TRACK_SEARCH_DISTANCE_PX:
                minimum_circle_score = BALL_MOTION_MIN_CIRCLE_SCORE
            else:
                minimum_circle_score = BALL_MIN_CIRCLE_SCORE

        if circle_score < minimum_circle_score:
            if diagnostics is not None:
                diagnostics["circle"] += 1
            continue

        axis_distance = point_to_line_distance(
            center, axis_start, axis_end)
        if axis_distance > max_axis_distance:
            if diagnostics is not None:
                diagnostics["axis"] += 1
            continue

        # Initial acquisition prefers the most circle-like candidate. During
        # tracking, a nearby motion-blurred candidate receives a continuity
        # bonus, while pixel count remains only a weak secondary factor.
        score = circle_score + 0.01 * math.sqrt(float(blob.pixels()))
        if (
            tracking_distance is not None and
            tracking_distance <= BALL_TRACK_SEARCH_DISTANCE_PX
        ):
            proximity = (
                1.0 -
                tracking_distance / BALL_TRACK_SEARCH_DISTANCE_PX
            )
            score += 2.0 + 2.0 * proximity
        candidates.append((
            score,
            blob,
            aspect_ratio,
            fill_ratio,
            circle_score,
            tracking_distance,
            axis_distance,
        ))

    if diagnostics is not None:
        diagnostics["accepted"] = len(candidates)
    if not candidates:
        return None, diagnostics
    candidates.sort(key=lambda item: item[0], reverse=True)
    (
        _,
        blob,
        aspect_ratio,
        fill_ratio,
        circle_score,
        tracking_distance,
        axis_distance,
    ) = candidates[0]
    if diagnostics is not None:
        diagnostics["selected_rect"] = blob.rect()
        diagnostics["selected_pixels"] = blob.pixels()
        diagnostics["selected_aspect"] = aspect_ratio
        diagnostics["selected_fill"] = fill_ratio
        diagnostics["selected_circle"] = circle_score
        if tracking_distance is not None:
            diagnostics["selected_track_dist"] = tracking_distance
        diagnostics["selected_axis_dist"] = axis_distance
    return blob, diagnostics


def corner_variants(corners):
    """Generate all cyclic and reversed orders of four rectangle corners."""
    points = [
        (float(point[0]), float(point[1]))
        for point in corners
    ]
    variants = []
    for ordered in (points, list(reversed(points))):
        for shift in range(4):
            variants.append(ordered[shift:] + ordered[:shift])
    return variants


def corner_match_cost(reference, candidate):
    total = 0.0
    for index in range(4):
        dx = candidate[index][0] - reference[index][0]
        dy = candidate[index][1] - reference[index][1]
        total += dx * dx + dy * dy
    return total


class RectangleTracker:
    """Smooth a rotated rectangle while preserving corner correspondence."""

    def __init__(self, alpha, max_jump_px, hold_frames):
        self.alpha = float(alpha)
        self.max_jump_px = float(max_jump_px)
        self.hold_frames = int(hold_frames)
        self.corners = None
        self.missing_frames = 0

    def _integer_corners(self):
        if self.corners is None:
            return None
        return [
            (int(round(point[0])), int(round(point[1])))
            for point in self.corners
        ]

    def update(self, measured_corners):
        if measured_corners is None:
            self.missing_frames += 1
            if (self.corners is not None and
                    self.missing_frames <= self.hold_frames):
                return self._integer_corners(), "HOLD"
            self.corners = None
            return None, "LOST"

        measured_variants = corner_variants(measured_corners)
        if self.corners is None:
            self.corners = measured_variants[0]
            self.missing_frames = 0
            return self._integer_corners(), "LOCK"

        aligned = min(
            measured_variants,
            key=lambda points: corner_match_cost(self.corners, points),
        )
        average_jump = math.sqrt(
            corner_match_cost(self.corners, aligned) / 4.0)

        if average_jump > self.max_jump_px:
            self.missing_frames += 1
            if self.missing_frames <= self.hold_frames:
                return self._integer_corners(), "REJECT"
            self.corners = aligned
            self.missing_frames = 0
            return self._integer_corners(), "RELOCK"

        inverse_alpha = 1.0 - self.alpha
        self.corners = [
            (
                inverse_alpha * self.corners[index][0] +
                self.alpha * aligned[index][0],
                inverse_alpha * self.corners[index][1] +
                self.alpha * aligned[index][1],
            )
            for index in range(4)
        ]
        self.missing_frames = 0
        return self._integer_corners(), "TRACK"


class PointTracker:
    """Smooth the ball centre and hold short detection dropouts."""

    def __init__(self, alpha, max_jump_px, hold_frames):
        self.alpha = float(alpha)
        self.max_jump_px = float(max_jump_px)
        self.hold_frames = int(hold_frames)
        self.point = None
        self.missing_frames = 0

    def _integer_point(self):
        if self.point is None:
            return None
        return (
            int(round(self.point[0])),
            int(round(self.point[1])),
        )

    def update(self, measurement):
        if measurement is None:
            self.missing_frames += 1
            if (self.point is not None and
                    self.missing_frames <= self.hold_frames):
                return self._integer_point(), "HOLD"
            self.point = None
            return None, "LOST"

        measured = (
            float(measurement[0]),
            float(measurement[1]),
        )
        if self.point is None:
            self.point = measured
            self.missing_frames = 0
            return self._integer_point(), "LOCK"

        center_jump = distance(self.point, measured)
        if center_jump > self.max_jump_px:
            self.missing_frames += 1
            if self.missing_frames <= self.hold_frames:
                return self._integer_point(), "REJECT"
            self.point = measured
            self.missing_frames = 0
            return self._integer_point(), "RELOCK"

        inverse_alpha = 1.0 - self.alpha
        self.point = (
            inverse_alpha * self.point[0] + self.alpha * measured[0],
            inverse_alpha * self.point[1] + self.alpha * measured[1],
        )
        self.missing_frames = 0
        return self._integer_point(), "TRACK"


def main():
    program_start_ms = time.ticks_ms()
    os.exitpoint(os.EXITPOINT_ENABLE)
    sensor = Sensor(width=IMAGE_W, height=IMAGE_H, fps=SENSOR_FPS)
    display_initialized = False
    media_initialized = False
    uart2 = None

    try:
        sensor.reset()
        sensor.set_hmirror(CAMERA_HMIRROR)
        sensor.set_vflip(CAMERA_VFLIP)
        sensor.set_framesize(width=IMAGE_W, height=IMAGE_H)
        sensor.set_pixformat(Sensor.RGB565)

        Display.init(
            Display.VIRT,
            width=DISPLAY_W,
            height=DISPLAY_H,
            fps=DISPLAY_FPS,
            to_ide=True,
            quality=DISPLAY_QUALITY,
        )
        display_initialized = True

        MediaManager.init()
        media_initialized = True
        sensor.run()
        # Initialize UART after all media devices so their setup cannot
        # overwrite the IO5/IO6 FPIOA mapping.
        uart2 = init_uart2()

        fps = time.clock()
        frame_count = 0
        uart_sequence = 0
        last_uart_frame = None
        last_uart_position_x10 = UART_INVALID_POSITION_X10
        last_uart_was_held = False
        last_valid_distance_mm = None
        ball_reference_point = None
        ball_reference_missing_frames = 0
        previous_tracker_state = None
        previous_ball_state = None
        canvas = image.Image(DISPLAY_W, DISPLAY_H, image.RGB565)
        camera_img = image.Image(PROCESS_W, PROCESS_H, image.RGB565)
        binary_img = image.Image(PROCESS_W, PROCESS_H, image.RGB565)
        calibration_img = image.Image(
            PROCESS_W, PROCESS_H, image.RGB565)
        rectangle_tracker = RectangleTracker(
            RECT_SMOOTH_ALPHA,
            RECT_MAX_CORNER_JUMP_PX,
            RECT_HOLD_FRAMES,
        )
        ball_tracker = PointTracker(
            BALL_SMOOTH_ALPHA,
            BALL_MAX_CENTER_JUMP_PX,
            BALL_HOLD_FRAMES,
        )

        print("Step 1: LAB threshold verification")
        print("UART2 TX=IO%d RX=IO%d baud=%d 8N1" % (
            UART_TX_PIN, UART_RX_PIN, UART_BAUDRATE))
        print(
            "UART frame: AA 55 seq left_mm_x10 int16 "
            "0 int16 conf flags crc8")
        print("bar_threshold =", LAB_THRESHOLD)
        print("ball_threshold =", BALL_BLACK_THRESHOLD)
        print(
            "sensor=%dx%d process_roi=%s process=%dx%d sensor_fps=%d "
            "display_fps=%d bar_length_mm=%.1f" %
            (
                IMAGE_W,
                IMAGE_H,
                str(PROCESS_ROI),
                PROCESS_W,
                PROCESS_H,
                SENSOR_FPS,
                DISPLAY_FPS,
                BAR_LENGTH_MM,
            ))
        print(
            "bar_filter pixels=%d area=%d long_min=%d short_max=%d "
            "aspect=%.2f..%.2f fill_min=%.2f" %
            (
                MIN_BLOB_PIXELS,
                MIN_BLOB_AREA,
                BAR_MIN_LONG_SIDE_PX,
                BAR_MAX_SHORT_SIDE_PX,
                BAR_MIN_ASPECT_RATIO,
                BAR_MAX_ASPECT_RATIO,
                BAR_MIN_FILL_RATIO,
            ))
        print(
            "ball_filter pixels=%d area=%d diameter=%d..%d "
            "short_min=%d aspect=%.2f..%.2f "
            "fill_min=%.2f circle=%.2f/%.2f track_px=%.0f "
            "axis_ratio=%.2f" %
            (
                BALL_MIN_PIXELS,
                BALL_MIN_AREA,
                BALL_MIN_DIAMETER_PX,
                BALL_MAX_DIAMETER_PX,
                BALL_MIN_SHORT_SIDE_PX,
                BALL_MIN_ASPECT_RATIO,
                BALL_MAX_ASPECT_RATIO,
                BALL_MIN_FILL_RATIO,
                BALL_MIN_CIRCLE_SCORE,
                BALL_MOTION_MIN_CIRCLE_SCORE,
                BALL_TRACK_SEARCH_DISTANCE_PX,
                BALL_MAX_AXIS_DISTANCE_RATIO,
            ))
        print("Expected result: bar=white, background+ball=black")

        while True:
            os.exitpoint()
            fps.tick()
            sensor_img = sensor.snapshot()
            camera_img.draw_image(
                sensor_img,
                -ROI_LEFT,
                -ROI_TOP,
            )

            blobs = camera_img.find_blobs(
                [LAB_THRESHOLD],
                roi=DETECT_ROI,
                pixels_threshold=MIN_BLOB_PIXELS,
                area_threshold=MIN_BLOB_AREA,
                merge=BLOB_MERGE,
                margin=BLOB_MERGE_MARGIN,
            )
            bar_blob, aspect_ratio, fill_ratio = select_bar_blob(blobs)

            binary_img.replace(camera_img)
            binary_img.binary([LAB_THRESHOLD])
            if BALL_BLACK_DILATE_SIZE > 0:
                binary_img.erode(BALL_BLACK_DILATE_SIZE)

            calibration_img.replace(camera_img)

            if bar_blob is not None:
                rect = bar_blob.rect()
                measured_corners = bar_blob.min_corners()
            else:
                rect = None
                measured_corners = None
                aspect_ratio = 0.0
                fill_ratio = 0.0

            corners, tracker_state = rectangle_tracker.update(
                measured_corners)
            rectangle_info = (
                rectangle_geometry(corners)
                if corners is not None else None
            )
            ball_distance_mm = None
            ball_blob = None
            ball_diagnostics = None

            if rectangle_info is not None:
                ball_search_roi = corners_bounding_roi(corners)
                ball_blobs = binary_img.find_blobs(
                    [BALL_BLACK_THRESHOLD],
                    roi=ball_search_roi,
                    pixels_threshold=BALL_MIN_PIXELS,
                    area_threshold=BALL_MIN_AREA,
                    merge=BALL_BLOB_MERGE,
                    margin=BALL_BLOB_MERGE_MARGIN,
                )
                ball_blob, ball_diagnostics = select_ball_blob(
                    ball_blobs,
                    corners,
                    rectangle_info,
                    ball_reference_point,
                )
                if ball_blob is not None:
                    ball_measurement = (
                        ball_blob.cx(),
                        ball_blob.cy(),
                    )
                    ball_reference_point = ball_measurement
                    ball_reference_missing_frames = 0
                else:
                    ball_measurement = None
                    ball_reference_missing_frames += 1
                    if (
                        ball_reference_missing_frames >
                        BALL_REFERENCE_HOLD_FRAMES
                    ):
                        ball_reference_point = None
                ball_point, ball_state = ball_tracker.update(
                    ball_measurement)

                draw_rotated_rectangle(
                    binary_img,
                    corners,
                    BAR_ROTATED_RECT_COLOR,
                    BAR_LINE_THICKNESS,
                )
                draw_rotated_rectangle(
                    calibration_img,
                    corners,
                    BAR_ROTATED_RECT_COLOR,
                    BAR_LINE_THICKNESS,
                )

                axis_start = rectangle_info["axis_points"][0]
                axis_end = rectangle_info["axis_points"][1]

                for corner in corners:
                    calibration_img.draw_circle(
                        int(corner[0]),
                        int(corner[1]),
                        BAR_CORNER_RADIUS,
                        color=BAR_CORNER_COLOR,
                        thickness=BAR_CORNER_THICKNESS,
                    )

                if ball_point is not None:
                    calibration_img.draw_circle(
                        ball_point[0],
                        ball_point[1],
                        BALL_POINT_RADIUS,
                        color=BALL_POINT_COLOR,
                        fill=True,
                    )
                    binary_img.draw_circle(
                        ball_point[0],
                        ball_point[1],
                        BALL_POINT_RADIUS,
                        color=BALL_POINT_COLOR,
                        fill=True,
                    )
                    ball_distance_mm = ball_distance_from_left_mm(
                        ball_point,
                        axis_start,
                        axis_end,
                    )
            else:
                corners = None
                ball_search_roi = None
                ball_reference_missing_frames += 1
                if (
                    ball_reference_missing_frames >
                    BALL_REFERENCE_HOLD_FRAMES
                ):
                    ball_reference_point = None
                ball_point, ball_state = ball_tracker.update(None)

            if ball_distance_mm is not None:
                last_valid_distance_mm = ball_distance_mm
                uart_distance_mm = ball_distance_mm
                uart_holding_last_valid = False
            elif (
                last_valid_distance_mm is not None and
                time.ticks_diff(
                    time.ticks_ms(), program_start_ms
                ) >= UART_LOSS_HOLD_DELAY_MS
            ):
                uart_distance_mm = last_valid_distance_mm
                uart_holding_last_valid = True
            else:
                uart_distance_mm = None
                uart_holding_last_valid = False

            if (
                UART_SEND_INTERVAL_FRAMES > 0 and
                frame_count % UART_SEND_INTERVAL_FRAMES == 0
            ):
                last_uart_frame = build_uart_distance_frame(
                    uart_sequence,
                    uart_distance_mm,
                    uart_holding_last_valid,
                )
                uart2.write(last_uart_frame)
                last_uart_position_x10 = (
                    int(round(uart_distance_mm * 10.0))
                    if uart_distance_mm is not None
                    else UART_INVALID_POSITION_X10
                )
                last_uart_was_held = uart_holding_last_valid
                uart_sequence = (uart_sequence + 1) & 0xFF

            if ball_distance_mm is None:
                distance_text = "DIST: --.- mm"
            else:
                distance_text = "DIST: %.1f mm" % ball_distance_mm
            calibration_img.draw_string_advanced(
                DISTANCE_TEXT_X,
                DISTANCE_TEXT_Y,
                DISTANCE_FONT_SIZE,
                distance_text,
                color=DISTANCE_TEXT_COLOR,
            )

            canvas.clear()
            canvas.draw_string_advanced(
                4, 2, TITLE_FONT_SIZE, "ORIGINAL", color=TEXT_COLOR)
            canvas.draw_string_advanced(
                PANEL_W + 4, 2, TITLE_FONT_SIZE, "BINARY",
                color=TEXT_COLOR)
            canvas.draw_string_advanced(
                PANEL_W * 2 + 4, 2, TITLE_FONT_SIZE, "CALIBRATION",
                color=TEXT_COLOR)
            canvas.draw_image(
                camera_img,
                PREVIEW_X_OFFSET,
                TITLE_HEIGHT + PREVIEW_Y_OFFSET,
                x_scale=PREVIEW_SCALE,
                y_scale=PREVIEW_SCALE,
            )
            canvas.draw_image(
                binary_img,
                PANEL_W + PREVIEW_X_OFFSET,
                TITLE_HEIGHT + PREVIEW_Y_OFFSET,
                x_scale=PREVIEW_SCALE,
                y_scale=PREVIEW_SCALE,
            )
            canvas.draw_image(
                calibration_img,
                PANEL_W * 2 + PREVIEW_X_OFFSET,
                TITLE_HEIGHT + PREVIEW_Y_OFFSET,
                x_scale=PREVIEW_SCALE,
                y_scale=PREVIEW_SCALE,
            )
            canvas.draw_line(
                PANEL_W - 1, 0, PANEL_W - 1, DISPLAY_H - 1,
                color=SEPARATOR_COLOR, thickness=1)
            canvas.draw_line(
                PANEL_W * 2 - 1, 0, PANEL_W * 2 - 1, DISPLAY_H - 1,
                color=SEPARATOR_COLOR, thickness=1)
            Display.show_image(
                canvas,
            )

            frame_count += 1
            periodic_log = (
                LOG_INTERVAL_FRAMES > 0 and
                frame_count % LOG_INTERVAL_FRAMES == 0
            )
            state_changed = (
                tracker_state != previous_tracker_state or
                ball_state != previous_ball_state
            )
            should_log = (
                DIAGNOSTIC_LOGS and
                (periodic_log or
                 (LOG_ON_STATE_CHANGE and state_changed))
            )
            if should_log:
                try:
                    free_memory = gc.mem_free()
                except BaseException:
                    free_memory = -1

                print(
                    "DIAG frame=%d fps=%.1f mem_free=%d "
                    "bar_blobs=%d bar_state=%s ball_state=%s" %
                    (
                        frame_count,
                        fps.fps(),
                        free_memory,
                        len(blobs),
                        tracker_state,
                        ball_state,
                    ))
                if rectangle_info is None:
                    print(
                        " BAR raw_rect=%s raw_corners=%s "
                        "tracked_corners=None" %
                        (
                            str(rect),
                            str(measured_corners),
                        ))
                    print(" BALL point=None distance_mm=None diagnostics=None")
                else:
                    axis_point_a = rectangle_info["axis_points"][0]
                    axis_point_b = rectangle_info["axis_points"][1]
                    if axis_point_a[0] <= axis_point_b[0]:
                        left_axis_point = axis_point_a
                        right_axis_point = axis_point_b
                    else:
                        left_axis_point = axis_point_b
                        right_axis_point = axis_point_a
                    axis_angle_deg = (
                        math.atan2(
                            right_axis_point[1] - left_axis_point[1],
                            right_axis_point[0] - left_axis_point[0],
                        ) * 180.0 / math.pi
                    )
                    print(
                        " BAR raw_rect=%s pixels=%d fill=%.3f "
                        "raw_corners=%s tracked_corners=%s center=%s "
                        "left=%s right=%s angle_deg=%.2f "
                        "long_px=%.2f short_px=%.2f aspect=%.3f" %
                        (
                            str(rect),
                            bar_blob.pixels() if bar_blob is not None else 0,
                            fill_ratio,
                            str(measured_corners),
                            str(corners),
                            str(rectangle_info["center"]),
                            str(left_axis_point),
                            str(right_axis_point),
                            axis_angle_deg,
                            rectangle_info["long_side"],
                            rectangle_info["short_side"],
                            rectangle_info["aspect_ratio"],
                        ))
                    print(
                        " BALL roi=%s raw_rect=%s point=%s "
                        "distance_mm=%s ratio=%s diagnostics=%s" %
                        (
                            str(ball_search_roi),
                            str(ball_blob.rect())
                            if ball_blob is not None else "None",
                            str(ball_point),
                            str(ball_distance_mm),
                            str(
                                ball_distance_mm / BAR_LENGTH_MM
                                if ball_distance_mm is not None else None
                            ),
                            str(ball_diagnostics),
                        ))

            previous_tracker_state = tracker_state
            previous_ball_state = ball_state
            if (
                not DIAGNOSTIC_LOGS and
                FPS_LOG_INTERVAL_FRAMES > 0 and
                frame_count % FPS_LOG_INTERVAL_FRAMES == 0
            ):
                print("fps=%.1f" % fps.fps())
            if (
                UART_LOG_INTERVAL_FRAMES > 0 and
                frame_count % UART_LOG_INTERVAL_FRAMES == 0 and
                last_uart_frame is not None
            ):
                if last_uart_position_x10 == UART_INVALID_POSITION_X10:
                    uart_distance_text = "invalid"
                else:
                    uart_distance_text = "%.1f" % (
                        last_uart_position_x10 / 10.0)
                print(
                    "UART2 left_mm=%s valid=%d hold=%d tx=%s" %
                    (
                        uart_distance_text,
                        (1 if
                         last_uart_position_x10 !=
                         UART_INVALID_POSITION_X10 else 0),
                        1 if last_uart_was_held else 0,
                        "".join(
                            "%02X" % value for value in last_uart_frame),
                    ))

            del sensor_img

            if (GC_INTERVAL_FRAMES > 0 and
                    frame_count % GC_INTERVAL_FRAMES == 0):
                gc.collect()

    except KeyboardInterrupt:
        print("user stopped threshold verification")
    except BaseException as exc:
        import sys
        sys.print_exception(exc)
    finally:
        if uart2 is not None:
            try:
                uart2.deinit()
            except BaseException:
                pass
        try:
            sensor.stop()
        except BaseException:
            pass
        if display_initialized:
            try:
                Display.deinit()
            except BaseException:
                pass
        time.sleep_ms(50)
        if media_initialized:
            try:
                MediaManager.deinit()
            except BaseException:
                pass


if __name__ == "__main__":
    main()
