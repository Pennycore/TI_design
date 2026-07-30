import gc
import os
import struct
import time

from machine import FPIOA, UART
from media.display import Display
from media.media import MediaManager
from media.sensor import Sensor


IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# Initial values measured from the current fixed camera view.
# ROD_LEFT_X maps to -125 mm, ROD_RIGHT_X maps to +125 mm.
ROD_LEFT_X = 16
ROD_RIGHT_X = 624
ROD_CENTER_Y = 306
ROD_ROI_HEIGHT = 116

# A short exposure reduces steel-ball motion blur. Add diffuse lighting if the
# image becomes too dark before increasing this value.
# Keep auto exposure during detector setup. Switch this back to True after
# fixed diffuse lighting is installed for high-speed tests.
USE_MANUAL_EXPOSURE = False
MANUAL_EXPOSURE_US = 10000

TARGET_MM = 0
TARGET_TOLERANCE_MM = 10

UART_BAUD = 115200
UART_ID = UART.UART2
K230_UART_TX_GPIO = 5
K230_UART_RX_GPIO = 6

MIN_RADIUS = 8
MAX_RADIUS = 20
EXPECTED_RADIUS = 12
MAX_CENTER_Y_ERROR = 28
TRACK_GATE_PIXELS = 90
CIRCLE_THRESHOLD = 1800
USE_LOCAL_CONTRAST_NORMALIZATION = True
HISTEQ_ADAPTIVE = False
HISTEQ_CLIP_LIMIT = 10
STABLE_FRAMES = 2
POSITION_FILTER_ALPHA = 0.72
VELOCITY_FILTER_BETA = 0.12
MAX_ABS_VELOCITY_MM_S = 1500.0
DEBUG_PRINT = True
DEBUG_PRINT_INTERVAL = 10
PERF_PRINT_INTERVAL = 30

SERIAL_SOF0 = 0xA5
SERIAL_SOF1 = 0x5A
MSG_ROD_BALL_POSITION = 0x12
ROD_FLAG_DETECTED = 0x01
ROD_FLAG_STABLE = 0x02
ROD_FLAG_ON_TARGET = 0x04

normalization_failure_reported = False


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
    header = bytes([SERIAL_SOF0, SERIAL_SOF1, msg_id & 0xFF, seq & 0xFF, len(payload) & 0xFF])
    return header + payload + bytes([crc8(header[2:] + payload)])


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def circle_value(circle, name, default=0):
    if isinstance(circle, dict):
        return circle.get(name, default)
    attr = getattr(circle, name, None)
    if attr is None:
        return default
    return attr() if callable(attr) else attr


def rod_roi():
    y = clamp(ROD_CENTER_Y - ROD_ROI_HEIGHT // 2, 0, IMAGE_HEIGHT - 1)
    h = clamp(ROD_ROI_HEIGHT, 1, IMAGE_HEIGHT - y)
    return (0, y, IMAGE_WIDTH, h)


def pixel_to_mm(x):
    span = max(1, ROD_RIGHT_X - ROD_LEFT_X)
    return int(round(((x - ROD_LEFT_X) * 250.0 / span) - 125.0))


def choose_best_circle(circles, expected_x=None):
    best = None
    best_score = -100000

    for circle in circles:
        x = int(circle_value(circle, "x"))
        y = int(circle_value(circle, "y"))
        radius = int(circle_value(circle, "r"))
        magnitude = int(circle_value(circle, "magnitude", 0))

        if radius < MIN_RADIUS or radius > MAX_RADIUS:
            continue
        if x < ROD_LEFT_X - 20 or x > ROD_RIGHT_X + 20:
            continue
        if abs(y - ROD_CENTER_Y) > MAX_CENTER_Y_ERROR:
            continue
        if expected_x is not None and abs(x - expected_x) > TRACK_GATE_PIXELS:
            continue

        y_penalty = abs(y - ROD_CENTER_Y) * 8
        track_penalty = 0
        if expected_x is not None:
            track_penalty = abs(x - expected_x) * 2
        radius_penalty = abs(radius - EXPECTED_RADIUS) * 80
        radius_bonus = radius * 20
        score = (
            magnitude
            + radius_bonus
            - y_penalty
            - track_penalty
            - radius_penalty
        )
        if score > best_score:
            best_score = score
            best = circle

    return best, best_score


def detect_ball(img, expected_x=None):
    global normalization_failure_reported
    roi = rod_roi()
    try:
        if USE_LOCAL_CONTRAST_NORMALIZATION:
            normalized = img.copy(roi)
            normalized.to_grayscale()
            normalized.histeq(
                adaptive=HISTEQ_ADAPTIVE,
                clip_limit=HISTEQ_CLIP_LIMIT,
            )
            local_circles = normalized.find_circles(
                threshold=CIRCLE_THRESHOLD,
                x_margin=8,
                y_margin=8,
                r_margin=4,
                r_min=MIN_RADIUS,
                r_max=MAX_RADIUS,
                r_step=1,
            )
            circles = []
            for circle in local_circles:
                circles.append({
                    "x": int(circle_value(circle, "x")) + roi[0],
                    "y": int(circle_value(circle, "y")) + roi[1],
                    "r": int(circle_value(circle, "r")),
                    "magnitude": int(circle_value(circle, "magnitude", 0)),
                })
        else:
            circles = img.find_circles(
                roi=roi,
                threshold=CIRCLE_THRESHOLD,
                x_margin=8,
                y_margin=8,
                r_margin=4,
                r_min=MIN_RADIUS,
                r_max=MAX_RADIUS,
                r_step=1,
            )
    except Exception as error:
        if USE_LOCAL_CONTRAST_NORMALIZATION and not normalization_failure_reported:
            print("ROI normalization unavailable; using raw image:", error)
            normalization_failure_reported = True
        try:
            circles = img.find_circles(
                roi=roi,
                threshold=CIRCLE_THRESHOLD,
                x_margin=8,
                y_margin=8,
                r_margin=4,
                r_min=MIN_RADIUS,
                r_max=MAX_RADIUS,
                r_step=1,
            )
        except Exception:
            circles = []

    circle, score = choose_best_circle(circles, expected_x)
    if circle is None:
        return None

    x = int(circle_value(circle, "x"))
    y = int(circle_value(circle, "y"))
    radius = int(circle_value(circle, "r"))
    magnitude = int(circle_value(circle, "magnitude", score))
    position_mm = pixel_to_mm(x)
    confidence = clamp(35 + (magnitude // 90) + radius * 2, 0, 100)

    return {
        "x": x,
        "y": y,
        "radius": radius,
        "position_mm": position_mm,
        "confidence": confidence,
    }


class PositionVelocityFilter:
    def __init__(self):
        self.initialized = False
        self.position_mm = 0.0
        self.velocity_mm_s = 0.0
        self.last_ms = 0

    def update(self, measured_position_mm, now_ms):
        if not self.initialized:
            self.initialized = True
            self.position_mm = float(measured_position_mm)
            self.velocity_mm_s = 0.0
            self.last_ms = now_ms
            return self.position_mm, self.velocity_mm_s

        elapsed_ms = time.ticks_diff(now_ms, self.last_ms)
        self.last_ms = now_ms
        dt = clamp(elapsed_ms / 1000.0, 0.01, 0.10)

        predicted_position = self.position_mm + self.velocity_mm_s * dt
        residual = measured_position_mm - predicted_position
        self.position_mm = predicted_position + POSITION_FILTER_ALPHA * residual
        self.velocity_mm_s += VELOCITY_FILTER_BETA * residual / dt
        self.velocity_mm_s = clamp(
            self.velocity_mm_s,
            -MAX_ABS_VELOCITY_MM_S,
            MAX_ABS_VELOCITY_MM_S,
        )
        return self.position_mm, self.velocity_mm_s


def position_payload(ball, stable):
    if ball is None:
        return struct.pack("<hhHBB", 0, TARGET_MM, 0, 0, 0)

    flags = ROD_FLAG_DETECTED
    if stable:
        flags |= ROD_FLAG_STABLE
    if abs(ball["position_mm"] - TARGET_MM) <= TARGET_TOLERANCE_MM:
        flags |= ROD_FLAG_ON_TARGET

    return struct.pack(
        "<hhHBB",
        int(ball["position_mm"]),
        int(TARGET_MM),
        int(ball["radius"]),
        int(ball["confidence"]),
        int(flags),
    )


def setup_uart():
    fpioa = FPIOA()
    fpioa.set_function(K230_UART_TX_GPIO, FPIOA.UART2_TXD)
    fpioa.set_function(K230_UART_RX_GPIO, FPIOA.UART2_RXD)
    return UART(UART_ID, UART_BAUD)


def setup_camera():
    sensor = Sensor()
    sensor.reset()
    try:
        sensor.set_framesize(width=IMAGE_WIDTH, height=IMAGE_HEIGHT)
    except TypeError:
        sensor.set_framesize(Sensor.QVGA)
    sensor.set_pixformat(Sensor.RGB565)

    manual_exposure_ready = False
    if USE_MANUAL_EXPOSURE:
        try:
            sensor.auto_exposure(False)
            manual_exposure_ready = True
        except Exception as error:
            print("Manual exposure unavailable:", error)
    else:
        try:
            sensor.auto_exposure(True)
            print("Auto exposure enabled")
        except Exception as error:
            print("Auto exposure unavailable:", error)

    try:
        Display.init(Display.VIRT, width=IMAGE_WIDTH, height=IMAGE_HEIGHT, to_ide=True)
    except TypeError:
        Display.init(Display.VIRT)
    MediaManager.init()
    sensor.run()

    if manual_exposure_ready:
        try:
            sensor.exposure(MANUAL_EXPOSURE_US)
            print("Manual exposure: %dus" % MANUAL_EXPOSURE_US)
        except Exception as error:
            print("Exposure setting failed:", error)
    return sensor


def draw_overlay(img, ball, stable):
    roi = rod_roi()
    center_x = (ROD_LEFT_X + ROD_RIGHT_X) // 2
    target_x = int(round(ROD_LEFT_X + (TARGET_MM + 125) * (ROD_RIGHT_X - ROD_LEFT_X) / 250.0))

    img.draw_rectangle(roi, color=(80, 160, 255), thickness=1)
    img.draw_line(ROD_LEFT_X, ROD_CENTER_Y, ROD_RIGHT_X, ROD_CENTER_Y, color=(255, 255, 0), thickness=2)
    img.draw_line(center_x, roi[1], center_x, roi[1] + roi[3], color=(255, 255, 255), thickness=1)
    img.draw_line(target_x, roi[1], target_x, roi[1] + roi[3], color=(255, 0, 255), thickness=1)

    if ball is not None:
        color = (0, 255, 0) if stable else (255, 160, 0)
        img.draw_circle(ball["x"], ball["y"], ball["radius"], color=color, thickness=2)
        img.draw_cross(ball["x"], ball["y"], color=color)
        img.draw_string_advanced(
            4,
            4,
            16,
            "pos=%dmm v=%d" % (
                ball["position_mm"],
                ball["velocity_mm_s"],
            ),
            color=color,
        )
    else:
        img.draw_string_advanced(
            4,
            4,
            16,
            "pos=none",
            color=(255, 0, 0),
        )


def main():
    uart = setup_uart()
    sensor = setup_camera()
    clock = time.clock()
    seq = 0
    frame_index = 0
    stable_count = 0
    last_ball = None
    state_filter = PositionVelocityFilter()

    print("Rod ball position monitor ready")
    print("UART2: TX GPIO%d, RX GPIO%d, %d baud" % (
        K230_UART_TX_GPIO,
        K230_UART_RX_GPIO,
        UART_BAUD,
    ))

    try:
        os.exitpoint(os.EXITPOINT_ENABLE)
        while True:
            os.exitpoint()
            clock.tick()
            img = sensor.snapshot()
            expected_x = last_ball["x"] if last_ball is not None else None
            ball = detect_ball(img, expected_x)

            if ball is None:
                stable_count = 0
            else:
                stable_count += 1
                raw_position_mm = ball["position_mm"]
                filtered_position, velocity = state_filter.update(
                    raw_position_mm,
                    time.ticks_ms(),
                )
                ball["raw_position_mm"] = raw_position_mm
                ball["position_mm"] = int(round(filtered_position))
                ball["velocity_mm_s"] = int(round(velocity))

            stable = stable_count >= STABLE_FRAMES
            uart.write(pack_frame(MSG_ROD_BALL_POSITION, seq, position_payload(ball, stable)))
            seq = (seq + 1) & 0xFF

            if DEBUG_PRINT and frame_index % DEBUG_PRINT_INTERVAL == 0:
                if ball is None:
                    print("ROD frame=%d pos=none fps=%.1f" % (
                        frame_index,
                        clock.fps(),
                    ))
                else:
                    print("ROD frame=%d x=%d raw=%dmm pos=%dmm v=%dmm/s r=%d conf=%d stable=%d" % (
                        frame_index,
                        ball["x"],
                        ball["raw_position_mm"],
                        ball["position_mm"],
                        ball["velocity_mm_s"],
                        ball["radius"],
                        ball["confidence"],
                        1 if stable else 0,
                    ))

            if frame_index % PERF_PRINT_INTERVAL == 0:
                print("ROD PERF fps=%.1f" % clock.fps())

            draw_overlay(img, ball, stable)
            Display.show_image(img)
            last_ball = ball
            frame_index += 1
            if frame_index % PERF_PRINT_INTERVAL == 0:
                gc.collect()
    finally:
        try:
            sensor.stop()
        except Exception:
            pass
        try:
            Display.deinit()
        except Exception:
            pass
        try:
            MediaManager.deinit()
        except Exception:
            pass


main()
