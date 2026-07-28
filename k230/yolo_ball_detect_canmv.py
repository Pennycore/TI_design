import struct
import time

from machine import FPIOA, UART
from media.display import Display
from media.media import MediaManager
from media.sensor import Sensor


IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
MODEL_INPUT_SIZE = 416
CENTER_X = IMAGE_WIDTH // 2

MODEL_PATH = "/sdcard/models/steel_ball_yolo11n.kmodel"
CONF_THRESHOLD = 0.45
NMS_THRESHOLD = 0.45
MAX_MULTI_BALLS = 4
CLOSE_BOX_HEIGHT = 80
STABLE_FRAMES = 3
TRACK_GATE_PIXELS = 30

UART_BAUD = 115200
UART_ID = UART.UART2
K230_UART_TX_GPIO = 5
K230_UART_RX_GPIO = 6

SERIAL_SOF0 = 0xA5
SERIAL_SOF1 = 0x5A
MSG_VISION_BALL = 0x10
MSG_VISION_MULTI_BALL = 0x11
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


def ball_payload(ball, stable):
    if ball is None:
        return struct.pack("<hhHhBB", 0, 0, 0, 0, 0, 0)

    flags = BALL_FLAG_DETECTED
    if stable:
        flags |= BALL_FLAG_STABLE
    if ball["height"] >= CLOSE_BOX_HEIGHT:
        flags |= BALL_FLAG_CLOSE

    radius = max(ball["width"], ball["height"]) // 2
    return struct.pack(
        "<hhHhBB",
        int(ball["cx"]),
        int(ball["cy"]),
        int(radius),
        int(ball["cx"] - CENTER_X),
        int(ball["confidence"] * 100),
        int(flags),
    )


def multi_ball_payload(balls):
    selected = balls[:MAX_MULTI_BALLS]
    payload = bytearray()
    payload.append(len(selected))
    for ball in selected:
        radius = max(ball["width"], ball["height"]) // 2
        payload += struct.pack(
            "<hhHB",
            int(ball["cx"]),
            int(ball["cy"]),
            int(radius),
            int(ball["confidence"] * 100),
        )
    return bytes(payload)


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


def load_detector():
    """
    TODO: Replace this adapter with the detector class provided by your CanMV/K230
    firmware package after converting steel_ball_yolo11n.onnx to kmodel.

    Expected return from detector.detect(img):
        [
            {"x1": int, "y1": int, "x2": int, "y2": int, "confidence": float},
            ...
        ]
    """
    raise RuntimeError(
        "Install the CanMV/K230 YOLO kmodel runtime adapter here. "
        "Model path: %s" % MODEL_PATH
    )


def normalize_detections(detections):
    balls = []
    for det in detections:
        conf = float(det.get("confidence", 0.0))
        if conf < CONF_THRESHOLD:
            continue

        x1 = int(det["x1"])
        y1 = int(det["y1"])
        x2 = int(det["x2"])
        y2 = int(det["y2"])
        width = max(0, x2 - x1)
        height = max(0, y2 - y1)
        if width <= 1 or height <= 1:
            continue

        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        balls.append(
            {
                "x1": x1,
                "y1": y1,
                "x2": x2,
                "y2": y2,
                "cx": cx,
                "cy": cy,
                "width": width,
                "height": height,
                "confidence": conf,
            }
        )

    balls.sort(key=lambda b: b["confidence"], reverse=True)
    return balls


def choose_best_detection(balls):
    best = None
    best_score = -1.0
    for ball in balls:
        center_penalty = abs(ball["cx"] - CENTER_X) / CENTER_X
        lower_bonus = ball["cy"] / IMAGE_HEIGHT
        area_bonus = (ball["width"] * ball["height"]) / (IMAGE_WIDTH * IMAGE_HEIGHT)
        score = ball["confidence"] + lower_bonus * 0.08 + area_bonus * 0.15 - center_penalty * 0.04
        if score > best_score:
            best_score = score
            best = ball
    return best


def main():
    uart = setup_uart()
    sensor = setup_camera()
    detector = load_detector()
    clock = time.clock()
    seq = 0
    last_ball = None
    stable_count = 0

    try:
        while True:
            clock.tick()
            img = sensor.snapshot()
            detections = detector.detect(img)
            balls = normalize_detections(detections)
            ball = choose_best_detection(balls)

            if ball is None:
                stable_count = 0
            elif last_ball is not None:
                dx = ball["cx"] - last_ball["cx"]
                dy = ball["cy"] - last_ball["cy"]
                if (dx * dx + dy * dy) <= (TRACK_GATE_PIXELS * TRACK_GATE_PIXELS):
                    stable_count += 1
                else:
                    stable_count = 1
            else:
                stable_count = 1

            stable = stable_count >= STABLE_FRAMES
            uart.write(pack_frame(MSG_VISION_BALL, seq, ball_payload(ball, stable)))
            seq = (seq + 1) & 0xFF
            uart.write(pack_frame(MSG_VISION_MULTI_BALL, seq, multi_ball_payload(balls)))
            seq = (seq + 1) & 0xFF

            for candidate in balls:
                color = (80, 180, 255)
                if ball is candidate:
                    color = (0, 255, 0) if stable else (255, 160, 0)
                img.draw_rectangle(
                    candidate["x1"],
                    candidate["y1"],
                    candidate["width"],
                    candidate["height"],
                    color=color,
                    thickness=2,
                )
                img.draw_cross(candidate["cx"], candidate["cy"], color=color)

            if ball is not None:
                last_ball = ball
            else:
                last_ball = None

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
