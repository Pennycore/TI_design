import struct
import time

from machine import FPIOA, UART
from media.display import Display
from media.media import MediaManager
from media.sensor import Sensor


IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
CENTER_X = IMAGE_WIDTH // 2

UART_BAUD = 115200
UART_ID = UART.UART2

# LCKFB Lite K230D 40Pin screenshot:
# physical pin 11 -> GPIO05 -> UART2_TXD
# physical pin 13 -> GPIO06 -> UART2_RXD
K230_UART_TX_GPIO = 5
K230_UART_RX_GPIO = 6

ROI = (0, 32, IMAGE_WIDTH, IMAGE_HEIGHT - 32)
MIN_RADIUS = 5
MAX_RADIUS = 48
CLOSE_RADIUS = 42
CLOSE_Y = 190
CIRCLE_THRESHOLD = 2500
TRACK_GATE_PIXELS = 28
STABLE_FRAMES = 3

SERIAL_SOF0 = 0xA5
SERIAL_SOF1 = 0x5A
MSG_VISION_BALL = 0x10
BALL_FLAG_DETECTED = 0x01
BALL_FLAG_STABLE = 0x02
BALL_FLAG_CLOSE = 0x04


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
    check = crc8(header[2:] + payload)
    return header + payload + bytes([check])


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def circle_value(circle, name, default=0):
    attr = getattr(circle, name, None)
    if attr is None:
        return default
    return attr() if callable(attr) else attr


def choose_best_circle(circles):
    best = None
    best_score = -100000

    for circle in circles:
        x = int(circle_value(circle, "x"))
        y = int(circle_value(circle, "y"))
        r = int(circle_value(circle, "r"))
        magnitude = int(circle_value(circle, "magnitude", 0))

        if r < MIN_RADIUS or r > MAX_RADIUS:
            continue

        score = magnitude + (y // 2) - (abs(x - CENTER_X) // 4)
        if score > best_score:
            best_score = score
            best = circle

    return best, best_score


def detect_ball(img):
    try:
        circles = img.find_circles(
            roi=ROI,
            threshold=CIRCLE_THRESHOLD,
            x_margin=10,
            y_margin=10,
            r_margin=8,
            r_min=MIN_RADIUS,
            r_max=MAX_RADIUS,
            r_step=2,
        )
    except Exception:
        circles = []

    circle, score = choose_best_circle(circles)
    if circle is None:
        return None

    x = int(circle_value(circle, "x"))
    y = int(circle_value(circle, "y"))
    r = int(circle_value(circle, "r"))
    magnitude = int(circle_value(circle, "magnitude", score))

    confidence = clamp(35 + (magnitude // 80) + (r * 2), 0, 100)
    return {
        "x": x,
        "y": y,
        "radius": r,
        "offset_x": x - CENTER_X,
        "confidence": confidence,
    }


def ball_payload(ball, stable):
    if ball is None:
        return struct.pack("<hhHhBB", 0, 0, 0, 0, 0, 0)

    flags = BALL_FLAG_DETECTED
    if stable:
        flags |= BALL_FLAG_STABLE
    if ball["radius"] >= CLOSE_RADIUS or ball["y"] >= CLOSE_Y:
        flags |= BALL_FLAG_CLOSE

    return struct.pack(
        "<hhHhBB",
        int(ball["x"]),
        int(ball["y"]),
        int(ball["radius"]),
        int(ball["offset_x"]),
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

    try:
        Display.init(Display.VIRT, width=IMAGE_WIDTH, height=IMAGE_HEIGHT, to_ide=True)
    except TypeError:
        Display.init(Display.VIRT)
    MediaManager.init()
    sensor.run()
    return sensor


def main():
    uart = setup_uart()
    sensor = setup_camera()
    clock = time.clock()
    seq = 0
    stable_count = 0
    last_ball = None

    try:
        while True:
            clock.tick()
            img = sensor.snapshot()
            ball = detect_ball(img)

            if ball is None:
                stable_count = 0
            elif last_ball is not None:
                dx = ball["x"] - last_ball["x"]
                dy = ball["y"] - last_ball["y"]
                if (dx * dx + dy * dy) <= (TRACK_GATE_PIXELS * TRACK_GATE_PIXELS):
                    stable_count += 1
                else:
                    stable_count = 1
            else:
                stable_count = 1

            stable = stable_count >= STABLE_FRAMES
            payload = ball_payload(ball, stable)
            uart.write(pack_frame(MSG_VISION_BALL, seq, payload))
            seq = (seq + 1) & 0xFF

            if ball is not None:
                color = (0, 255, 0) if stable else (255, 160, 0)
                img.draw_circle(ball["x"], ball["y"], ball["radius"], color=color, thickness=2)
                img.draw_cross(ball["x"], ball["y"], color=color)
                last_ball = ball
            else:
                last_ball = None

            img.draw_rectangle(ROI, color=(80, 160, 255), thickness=1)
            Display.show_image(img)
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
