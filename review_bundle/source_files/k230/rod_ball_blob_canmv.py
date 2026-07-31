import gc
import os
import struct
import sys
import time

from machine import FPIOA, UART
from media.display import Display
from media.media import MediaManager
from media.sensor import Sensor


IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# Calibrated from the current fixed camera view.
ROD_LEFT_X = 16
ROD_RIGHT_X = 624
ROD_CENTER_Y = 306
DETECT_ROI_HEIGHT = 44

# Three-point calibration measured with the camera in its final fixed position.
# Piecewise scaling compensates for perspective/lens asymmetry around O.
CAL_X_NEG_50_MM = 195
CAL_X_ZERO_MM = 315
CAL_X_POS_50_MM = 442

TARGET_MM = 0
TARGET_TOLERANCE_MM = 10

# Dedicated K230 <-> MSPM0 link. Do not reuse these pins for the video link.
UART_BAUD = 115200
UART_ID = UART.UART2
K230_UART_TX_GPIO = 5
K230_UART_RX_GPIO = 6

SERIAL_SOF0 = 0xA5
SERIAL_SOF1 = 0x5A
MSG_ROD_BALL_POSITION = 0x12
ROD_FLAG_DETECTED = 0x01
ROD_FLAG_STABLE = 0x02
ROD_FLAG_ON_TARGET = 0x04
ROD_FLAG_PREDICTED = 0x08

# The rail is bright and the steel ball contains a darker, nearly neutral area.
# L is selected relative to the current rail brightness. A/B reject colored marks.
LAB_A_MIN = -45
LAB_A_MAX = 45
LAB_B_MIN = -45
LAB_B_MAX = 45
DARK_L_OFFSET = 14
DARK_L_MIN = 8
DARK_L_MAX = 75
FALLBACK_L_MAX = 55

# Blob geometry at 640x480. Motion blur is allowed to stretch the ball in X.
MIN_BLOB_WIDTH = 5
MAX_BLOB_WIDTH = 68
MIN_BLOB_HEIGHT = 5
MAX_BLOB_HEIGHT = 60
# Measured from the final 640x480 camera view. Using the old 24 px value
# favored narrow rail marks over the actual 38-42 px ball image.
EXPECTED_DIAMETER = 38
MAX_ASPECT_PERCENT = 280
MIN_FILL_PERCENT = 55
MIN_CANDIDATE_CONFIDENCE = 20
REACQUIRE_MIN_CONFIDENCE = 20
MAX_CENTER_Y_ERROR = 22
# The ball can stop near the end caps. Edge candidates are allowed, but they
# must be ball-like; end-cap shadows often look like tall, thin bars.
ROD_EDGE_EXCLUSION = 36
EDGE_MAX_ASPECT_PERCENT = 220
EDGE_MIN_SHORT_SIDE = 8
EDGE_FOLLOW_GATE_PIXELS = 80
EDGE_SCORE_PENALTY = 90

BLOB_AREA_THRESHOLD = 18
BLOB_PIXELS_THRESHOLD = 12
# Keep nearby marks separate. Merging can chain the rail shadow, scale marks,
# and the ball into one oversized blob.
BLOB_MERGE_MARGIN = 0

# Constant-velocity alpha-beta tracker.
TRACK_ALPHA = 0.86
TRACK_BETA = 0.18
TRACK_GATE_PIXELS = 140
TRACK_CONFIRM_HITS = 3
MAX_TRACK_MISSES = 6
MAX_ABS_VELOCITY_PX_S = 2500.0

# Startup/reacquire latch: the ball may begin at any judge-specified position,
# but a single-frame blob is not trusted until it repeats nearby.
ACQUIRE_CONFIRM_HITS = 3
ACQUIRE_GATE_PIXELS = 28
ACQUIRE_EDGE_MARGIN = 18
ACQUIRE_MIN_SHORT_SIDE = 12
ACQUIRE_MAX_ASPECT_PERCENT = 220

DEBUG_PRINT_INTERVAL = 15
GC_INTERVAL = 120
DRAW_DEBUG_BLOBS = False


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def object_value(obj, name, default=0):
    attr = getattr(obj, name, None)
    if attr is None:
        return default
    return attr() if callable(attr) else attr


def detection_roi():
    y = clamp(
        ROD_CENTER_Y - DETECT_ROI_HEIGHT // 2,
        0,
        IMAGE_HEIGHT - 1,
    )
    height = clamp(DETECT_ROI_HEIGHT, 1, IMAGE_HEIGHT - y)
    return (ROD_LEFT_X, y, ROD_RIGHT_X - ROD_LEFT_X, height)


def pixel_to_mm(x):
    if x <= CAL_X_ZERO_MM:
        span = max(1, CAL_X_ZERO_MM - CAL_X_NEG_50_MM)
        return (x - CAL_X_ZERO_MM) * 50.0 / span

    span = max(1, CAL_X_POS_50_MM - CAL_X_ZERO_MM)
    return (x - CAL_X_ZERO_MM) * 50.0 / span


def velocity_px_to_mm_s(velocity_px_s, x):
    if x <= CAL_X_ZERO_MM:
        span = max(1, CAL_X_ZERO_MM - CAL_X_NEG_50_MM)
    else:
        span = max(1, CAL_X_POS_50_MM - CAL_X_ZERO_MM)
    return velocity_px_s * 50.0 / span


def crc8(data):
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def pack_frame(msg_id, seq, payload):
    header = bytes([
        SERIAL_SOF0,
        SERIAL_SOF1,
        msg_id & 0xFF,
        seq & 0xFF,
        len(payload) & 0xFF,
    ])
    return header + payload + bytes([crc8(header[2:] + payload)])


def rod_position_payload(track, ball):
    if track is None:
        return struct.pack("<hhHBB", 0, 0, 0, 0, 0)

    flags = 0
    if ball is not None:
        flags |= ROD_FLAG_DETECTED
    if track["locked"]:
        flags |= ROD_FLAG_STABLE
    if track["coasting"]:
        flags |= ROD_FLAG_PREDICTED
    if (
        ball is not None
        and track["locked"]
        and abs(track["position_mm"] - TARGET_MM) <= TARGET_TOLERANCE_MM
    ):
        flags |= ROD_FLAG_ON_TARGET

    confidence = ball["confidence"] if ball is not None else 0
    return struct.pack(
        "<hhHBB",
        int(clamp(track["position_mm"], -32768, 32767)),
        int(clamp(track["velocity_mm_s"], -32768, 32767)),
        int(clamp(track["x"], 0, 65535)),
        int(clamp(confidence, 0, 100)),
        flags,
    )


def normalized_l_value(value):
    value = int(value)
    if value > 100:
        value = int(round(value * 100.0 / 255.0))
    return clamp(value, 0, 100)


def adaptive_lab_threshold(img, roi):
    try:
        statistics = img.get_statistics(roi=roi)
        rail_l = normalized_l_value(
            object_value(statistics, "l_median", FALLBACK_L_MAX + DARK_L_OFFSET)
        )
        l_max = clamp(rail_l - DARK_L_OFFSET, DARK_L_MIN, DARK_L_MAX)
    except Exception:
        rail_l = FALLBACK_L_MAX + DARK_L_OFFSET
        l_max = FALLBACK_L_MAX

    threshold = (
        0,
        int(l_max),
        LAB_A_MIN,
        LAB_A_MAX,
        LAB_B_MIN,
        LAB_B_MAX,
    )
    return threshold, rail_l, l_max


def blob_candidate(blob, expected_x=None):
    x = int(object_value(blob, "x"))
    y = int(object_value(blob, "y"))
    width = int(object_value(blob, "w"))
    height = int(object_value(blob, "h"))
    center_x = int(object_value(blob, "cx", x + width // 2))
    center_y = int(object_value(blob, "cy", y + height // 2))
    pixels = int(object_value(blob, "pixels", 0))

    area = max(1, width * height)
    fill_percent = clamp(pixels * 100 // area, 0, 100)
    detail = {
        "rect": (x, y, width, height),
        "x": center_x,
        "y": center_y,
        "width": width,
        "height": height,
        "pixels": pixels,
        "fill_percent": fill_percent,
        "center_y_error": abs(center_y - ROD_CENTER_Y),
        "reject": "ok",
    }

    if width < MIN_BLOB_WIDTH or width > MAX_BLOB_WIDTH:
        detail["reject"] = "width"
        return None, detail
    if height < MIN_BLOB_HEIGHT or height > MAX_BLOB_HEIGHT:
        detail["reject"] = "height"
        return None, detail
    in_edge_zone = (
        center_x < ROD_LEFT_X + ROD_EDGE_EXCLUSION
        or center_x > ROD_RIGHT_X - ROD_EDGE_EXCLUSION
    )
    center_y_error = abs(center_y - ROD_CENTER_Y)
    if center_y_error > MAX_CENTER_Y_ERROR:
        detail["reject"] = "y"
        return None, detail

    short_side = max(1, min(width, height))
    long_side = max(width, height)
    aspect_percent = long_side * 100 // short_side
    if aspect_percent > MAX_ASPECT_PERCENT:
        detail["reject"] = "aspect"
        return None, detail
    if in_edge_zone:
        if short_side < EDGE_MIN_SHORT_SIDE:
            detail["reject"] = "edge_size"
            return None, detail
        if aspect_percent > EDGE_MAX_ASPECT_PERCENT:
            detail["reject"] = "edge_aspect"
            return None, detail

    if fill_percent < MIN_FILL_PERCENT:
        detail["reject"] = "fill"
        return None, detail

    track_error = 0
    if expected_x is not None:
        track_error = abs(center_x - expected_x)
        if track_error > TRACK_GATE_PIXELS:
            detail["reject"] = "gate"
            return None, detail

    size_error = abs(width - EXPECTED_DIAMETER) + abs(height - EXPECTED_DIAMETER)
    aspect_error = aspect_percent - 100
    edge_penalty = 0
    if in_edge_zone:
        edge_penalty = EDGE_SCORE_PENALTY
        if expected_x is not None and track_error <= EDGE_FOLLOW_GATE_PIXELS:
            edge_penalty = EDGE_SCORE_PENALTY // 3

    score = (
        1200
        - center_y_error * 9
        - size_error * 8
        - aspect_error * 2
        - track_error * 3
        - edge_penalty
        + min(pixels, 400) // 2
        + fill_percent
    )

    undersize_error = (
        max(0, EXPECTED_DIAMETER - width)
        + max(0, EXPECTED_DIAMETER - height)
    )
    oversize_error = (
        max(0, width - EXPECTED_DIAMETER)
        + max(0, height - EXPECTED_DIAMETER)
    )
    confidence = clamp(
        100
        - center_y_error * 2
        # A steel-ball highlight can split the dark LAB blob into two halves.
        # Keep the partial ball usable; low-fill rail shadows are rejected
        # separately by MIN_FILL_PERCENT.
        - undersize_error * 2
        - oversize_error
        - aspect_error // 3
        + fill_percent // 4,
        0,
        100,
    )
    detail["confidence"] = confidence
    if confidence < MIN_CANDIDATE_CONFIDENCE:
        detail["reject"] = "conf"
        return None, detail
    radius = max(3, int(round((width + height) / 4.0)))
    candidate = {
        "rect": (x, y, width, height),
        "x": center_x,
        "y": center_y,
        "width": width,
        "height": height,
        "radius": radius,
        "pixels": pixels,
        "fill_percent": fill_percent,
        "score": score,
        "confidence": confidence,
        "edge_zone": in_edge_zone,
        "acquire_ready": (
            center_x >= ROD_LEFT_X + ACQUIRE_EDGE_MARGIN
            and center_x <= ROD_RIGHT_X - ACQUIRE_EDGE_MARGIN
            and short_side >= ACQUIRE_MIN_SHORT_SIDE
            and aspect_percent <= ACQUIRE_MAX_ASPECT_PERCENT
            and confidence >= REACQUIRE_MIN_CONFIDENCE
        ),
    }
    return candidate, detail


def nearest_rejected_blob(details):
    nearest = None
    nearest_rank = None
    for detail in details:
        edge_penalty = 0
        if detail["reject"] == "edge":
            edge_penalty = 200
        rank = (
            detail["center_y_error"] * 4
            + abs(detail["width"] - EXPECTED_DIAMETER)
            + abs(detail["height"] - EXPECTED_DIAMETER)
            + edge_penalty
        )
        if nearest is None or rank < nearest_rank:
            nearest = detail
            nearest_rank = rank
    return nearest


def detect_ball(img, expected_x=None):
    roi = detection_roi()
    threshold, rail_l, l_max = adaptive_lab_threshold(img, roi)

    try:
        blobs = img.find_blobs(
            [threshold],
            roi=roi,
            x_stride=2,
            y_stride=1,
            area_threshold=BLOB_AREA_THRESHOLD,
            pixels_threshold=BLOB_PIXELS_THRESHOLD,
            merge=False,
            margin=BLOB_MERGE_MARGIN,
        )
    except Exception as error:
        print("find_blobs failed:", error)
        blobs = []

    candidates = []
    raw_details = []
    for blob in blobs:
        candidate, detail = blob_candidate(blob, expected_x)
        raw_details.append(detail)
        if candidate is not None:
            candidates.append(candidate)

    if expected_x is not None:
        selection_pool = candidates
    else:
        # End caps are stationary, high-contrast false targets. They may still
        # be followed after a valid lock, but cannot establish a new lock.
        selection_pool = [
            candidate
            for candidate in candidates
            if candidate["acquire_ready"]
        ]

    best = None
    for candidate in selection_pool:
        if best is None or candidate["score"] > best["score"]:
            best = candidate
    probe = None
    for candidate in candidates:
        if probe is None or candidate["score"] > probe["score"]:
            probe = candidate

    diagnostics = {
        "roi": roi,
        "rail_l": rail_l,
        "l_max": l_max,
        "raw_blob_count": len(blobs),
        "raw_rects": [detail["rect"] for detail in raw_details],
        "nearest_raw": nearest_rejected_blob(raw_details),
        "candidates": candidates,
        "probe": probe,
        "acquire_count": sum(
            1 for candidate in candidates if candidate["acquire_ready"]
        ),
    }
    return best, diagnostics


class AcquireLatch:
    def __init__(self):
        self.reset()

    def reset(self):
        self.x = None
        self.hits = 0

    def update(self, measured_x):
        if measured_x is None:
            self.reset()
            return None

        if self.x is None or abs(measured_x - self.x) > ACQUIRE_GATE_PIXELS:
            self.x = float(measured_x)
            self.hits = 1
            return None

        self.x = self.x * 0.65 + float(measured_x) * 0.35
        self.hits += 1
        if self.hits >= ACQUIRE_CONFIRM_HITS:
            return int(round(self.x))
        return None


class AlphaBetaTracker:
    def __init__(self):
        self.initialized = False
        self.locked = False
        self.x = 0.0
        self.velocity_px_s = 0.0
        self.last_ms = 0
        self.hits = 0
        self.misses = 0

    def reset(self):
        self.initialized = False
        self.locked = False
        self.velocity_px_s = 0.0
        self.hits = 0
        self.misses = 0

    def prediction(self, now_ms):
        if not self.initialized:
            return None
        elapsed_ms = time.ticks_diff(now_ms, self.last_ms)
        dt = clamp(elapsed_ms / 1000.0, 0.0, 0.10)
        return self.x + self.velocity_px_s * dt

    def update(self, measured_x, now_ms):
        if not self.initialized:
            if measured_x is None:
                return None
            self.initialized = True
            self.x = float(measured_x)
            self.velocity_px_s = 0.0
            self.last_ms = now_ms
            self.hits = 1
            self.misses = 0
            return self.state(False)

        elapsed_ms = time.ticks_diff(now_ms, self.last_ms)
        dt = clamp(elapsed_ms / 1000.0, 0.01, 0.10)
        predicted_x = self.x + self.velocity_px_s * dt
        self.last_ms = now_ms

        if measured_x is None:
            self.x = predicted_x
            self.misses += 1
            self.hits = 0
            if self.misses > MAX_TRACK_MISSES:
                self.reset()
                return None
            return self.state(True)

        residual = measured_x - predicted_x
        self.x = predicted_x + TRACK_ALPHA * residual
        self.velocity_px_s += TRACK_BETA * residual / dt
        self.velocity_px_s = clamp(
            self.velocity_px_s,
            -MAX_ABS_VELOCITY_PX_S,
            MAX_ABS_VELOCITY_PX_S,
        )
        self.misses = 0
        self.hits += 1
        if self.hits >= TRACK_CONFIRM_HITS:
            self.locked = True
        return self.state(False)

    def state(self, coasting):
        return {
            "x": int(round(self.x)),
            "position_mm": int(round(pixel_to_mm(self.x))),
            "velocity_mm_s": int(round(
                velocity_px_to_mm_s(self.velocity_px_s, self.x)
            )),
            "locked": self.locked,
            "coasting": coasting,
            "misses": self.misses,
        }


def setup_uart():
    fpioa = FPIOA()
    fpioa.set_function(K230_UART_TX_GPIO, FPIOA.UART2_TXD)
    fpioa.set_function(K230_UART_RX_GPIO, FPIOA.UART2_RXD)
    return UART(UART_ID, UART_BAUD)


def setup_camera():
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=IMAGE_WIDTH, height=IMAGE_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)

    try:
        sensor.auto_exposure(True)
        print("Auto exposure enabled")
    except Exception as error:
        print("Auto exposure warning:", error)

    try:
        Display.init(
            Display.VIRT,
            width=IMAGE_WIDTH,
            height=IMAGE_HEIGHT,
            to_ide=True,
        )
    except TypeError:
        Display.init(Display.VIRT)

    MediaManager.init()
    sensor.run()
    return sensor


def draw_overlay(img, ball, diagnostics, track, fps):
    roi = diagnostics["roi"]
    img.draw_rectangle(roi, color=(80, 160, 255), thickness=1)
    img.draw_line(
        ROD_LEFT_X,
        ROD_CENTER_Y,
        ROD_RIGHT_X,
        ROD_CENTER_Y,
        color=(255, 255, 0),
        thickness=1,
    )

    for mark_x in (CAL_X_NEG_50_MM, CAL_X_ZERO_MM, CAL_X_POS_50_MM):
        img.draw_line(
            mark_x,
            roi[1],
            mark_x,
            roi[1] + roi[3] - 1,
            color=(0, 220, 220),
            thickness=1,
        )

    nearest_raw = diagnostics["nearest_raw"]
    if DRAW_DEBUG_BLOBS:
        for raw_rect in diagnostics["raw_rects"]:
            img.draw_rectangle(
                raw_rect,
                color=(255, 80, 80),
                thickness=1,
            )

        if ball is None and nearest_raw is not None:
            img.draw_rectangle(
                nearest_raw["rect"],
                color=(255, 0, 255),
                thickness=2,
            )

        for candidate in diagnostics["candidates"]:
            img.draw_rectangle(
                candidate["rect"],
                color=(80, 120, 255),
                thickness=1,
            )

    if ball is not None and track is not None:
        img.draw_rectangle(ball["rect"], color=(0, 255, 0), thickness=2)
        img.draw_cross(ball["x"], ball["y"], color=(0, 255, 0))

    if track is not None:
        if not track["locked"]:
            track_color = (0, 180, 255)
            status = "acquire"
        elif track["coasting"]:
            track_color = (255, 180, 0)
            status = "coast"
        else:
            track_color = (0, 255, 0)
            status = "track"
        img.draw_cross(track["x"], ROD_CENTER_Y, color=track_color, size=12)
        line_one = "%s x=%d pos=%dmm v=%d" % (
            status,
            track["x"],
            track["position_mm"],
            track["velocity_mm_s"],
        )
    else:
        line_one = "searching"

    line_two = "fps=%.1f L=%d/%d blobs=%d cand=%d acq=%d" % (
        fps,
        diagnostics["rail_l"],
        diagnostics["l_max"],
        diagnostics["raw_blob_count"],
        len(diagnostics["candidates"]),
        diagnostics["acquire_count"],
    )
    img.draw_string_advanced(4, 4, 16, line_one, color=(0, 255, 0))
    img.draw_string_advanced(4, 24, 16, line_two, color=(255, 255, 255))
    probe = diagnostics["probe"]
    if track is None and probe is not None:
        line_three = "probe x=%d %dx%d fill=%d conf=%d ready=%d" % (
            probe["x"],
            probe["width"],
            probe["height"],
            probe["fill_percent"],
            probe["confidence"],
            1 if probe["acquire_ready"] else 0,
        )
        img.draw_string_advanced(4, 44, 16, line_three, color=(255, 220, 80))
    elif DRAW_DEBUG_BLOBS and ball is None and nearest_raw is not None:
        line_three = "near x=%d %dx%d f=%d reject=%s" % (
            nearest_raw["x"],
            nearest_raw["width"],
            nearest_raw["height"],
            nearest_raw["fill_percent"],
            nearest_raw["reject"],
        )
        img.draw_string_advanced(4, 44, 16, line_three, color=(255, 0, 255))


def main():
    sensor = None
    uart = None
    clock = time.clock()
    tracker = AlphaBetaTracker()
    acquirer = AcquireLatch()
    frame_index = 0
    seq = 0

    try:
        uart = setup_uart()
        sensor = setup_camera()
        os.exitpoint(os.EXITPOINT_ENABLE)
        print("ROI LAB blob detector ready")
        print("Calibration: x=%d -> -50mm, x=%d -> 0mm, x=%d -> +50mm" % (
            CAL_X_NEG_50_MM,
            CAL_X_ZERO_MM,
            CAL_X_POS_50_MM,
        ))
        print("UART2: TX GPIO%d, RX GPIO%d, %d baud, msg=0x%02X" % (
            K230_UART_TX_GPIO,
            K230_UART_RX_GPIO,
            UART_BAUD,
            MSG_ROD_BALL_POSITION,
        ))
        print("WiFi is not enabled in this detector test")

        while True:
            os.exitpoint()
            clock.tick()
            img = sensor.snapshot()
            now_ms = time.ticks_ms()
            expected_x = tracker.prediction(now_ms) if tracker.locked else None
            ball, diagnostics = detect_ball(img, expected_x)
            measured_x = ball["x"] if ball is not None else None
            if not tracker.initialized:
                confirmed_x = acquirer.update(measured_x)
                track = tracker.update(confirmed_x, now_ms)
            else:
                track = tracker.update(measured_x, now_ms)
                if track is None:
                    acquirer.reset()
            uart.write(pack_frame(
                MSG_ROD_BALL_POSITION,
                seq,
                rod_position_payload(track, ball),
            ))
            seq = (seq + 1) & 0xFF

            if frame_index % DEBUG_PRINT_INTERVAL == 0:
                if ball is None:
                    nearest_raw = diagnostics["nearest_raw"]
                    if nearest_raw is None:
                        nearest_text = "near=none"
                    else:
                        nearest_text = (
                            "near=x%d,y%d wh=%dx%d dy=%d fill=%d reject=%s"
                            % (
                                nearest_raw["x"],
                                nearest_raw["y"],
                                nearest_raw["width"],
                                nearest_raw["height"],
                                nearest_raw["center_y_error"],
                                nearest_raw["fill_percent"],
                                nearest_raw["reject"],
                            )
                        )
                    print(
                        "BLOB frame=%d fps=%.1f L=%d/%d raw=%d cand=%d acq=%d "
                        "ball=none %s"
                        % (
                            frame_index,
                            clock.fps(),
                            diagnostics["rail_l"],
                            diagnostics["l_max"],
                            diagnostics["raw_blob_count"],
                            len(diagnostics["candidates"]),
                            diagnostics["acquire_count"],
                            nearest_text,
                        )
                    )
                else:
                    position_mm = track["position_mm"] if track is not None else 0
                    velocity_mm_s = track["velocity_mm_s"] if track is not None else 0
                    print(
                        "BLOB frame=%d fps=%.1f x=%d pos=%dmm v=%dmm/s "
                        "wh=%dx%d fill=%d conf=%d raw=%d cand=%d acq=%d"
                        % (
                            frame_index,
                            clock.fps(),
                            ball["x"],
                            position_mm,
                            velocity_mm_s,
                            ball["width"],
                            ball["height"],
                            ball["fill_percent"],
                            ball["confidence"],
                            diagnostics["raw_blob_count"],
                            len(diagnostics["candidates"]),
                            diagnostics["acquire_count"],
                        )
                    )

            draw_overlay(img, ball, diagnostics, track, clock.fps())
            Display.show_image(img)
            frame_index += 1
            if frame_index % GC_INTERVAL == 0:
                gc.collect()
    except KeyboardInterrupt:
        print("Stopped by user")
    except BaseException as error:
        sys.print_exception(error)
    finally:
        if sensor is not None:
            try:
                sensor.stop()
            except BaseException:
                pass
        try:
            Display.deinit()
        except BaseException:
            pass
        try:
            MediaManager.deinit()
        except BaseException:
            pass
        if uart is not None:
            try:
                uart.deinit()
            except BaseException:
                pass
        gc.collect()


if __name__ == "__main__":
    main()
