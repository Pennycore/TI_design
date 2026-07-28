import gc
import struct
import sys

import aidemo
import image
import nncase_runtime as nn
import ulab.numpy as np
from machine import FPIOA, UART

from libs.AI2D import Ai2d
from libs.AIBase import AIBase
from libs.PipeLine import PipeLine
from libs.Utils import ALIGN_UP, ScopedTiming, get_colors, letterbox_pad_param


# The camera frame is letterboxed to 416x416 by AI2D before inference.
RGB888P_SIZE = [640, 480]
MODEL_INPUT_SIZE = [416, 416]
DISPLAY_MODE = "virt"  # Use "hdmi" when an HDMI monitor is connected.
DISPLAY_SIZE = [800, 480]

MODEL_PATH = "/sdcard/examples/kmodel/steel_ball_yolo11n.kmodel"
LABELS = ["steel_ball"]
CONF_THRESHOLD = 0.35
NMS_THRESHOLD = 0.45
MAX_BOXES_NUM = 30

MAX_MULTI_BALLS = 4
CLOSE_BOX_HEIGHT = 120
STABLE_FRAMES = 3
TRACK_GATE_PIXELS = 40

UART_BAUD = 115200
UART_ID = 2
K230_UART_TX_GPIO = 5
K230_UART_RX_GPIO = 6

SERIAL_SOF0 = 0xA5
SERIAL_SOF1 = 0x5A
MSG_VISION_BALL = 0x10
MSG_VISION_MULTI_BALL = 0x11
BALL_FLAG_DETECTED = 0x01
BALL_FLAG_STABLE = 0x02
BALL_FLAG_CLOSE = 0x04


class SteelBallDetectionApp(AIBase):
    def __init__(
        self,
        kmodel_path,
        model_input_size,
        rgb888p_size,
        display_size,
        confidence_threshold=0.35,
        nms_threshold=0.45,
        max_boxes_num=30,
        debug_mode=0,
    ):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.model_input_size = model_input_size
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.max_boxes_num = max_boxes_num
        self.debug_mode = debug_mode
        self.colors = get_colors(len(LABELS))

        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.uint8,
        )

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right, self.scale = letterbox_pad_param(
                self.rgb888p_size, self.model_input_size
            )
            self.ai2d.pad(
                [0, 0, 0, 0, top, bottom, left, right],
                0,
                [128, 128, 128],
            )
            self.ai2d.resize(
                nn.interp_method.tf_bilinear,
                nn.interp_mode.half_pixel,
            )
            self.ai2d.build(
                [1, 3, ai2d_input_size[1], ai2d_input_size[0]],
                [1, 3, self.model_input_size[1], self.model_input_size[0]],
            )

    def preprocess(self, input_np):
        with ScopedTiming("preprocess", self.debug_mode > 0):
            return [self.ai2d.run(input_np)]

    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            output = results[0][0].transpose()
            return aidemo.yolov8_det_postprocess(
                output.copy(),
                [self.rgb888p_size[1], self.rgb888p_size[0]],
                [self.model_input_size[1], self.model_input_size[0]],
                [self.display_size[1], self.display_size[0]],
                len(LABELS),
                self.confidence_threshold,
                self.nms_threshold,
                self.max_boxes_num,
            )

    def to_ball_list(self, detections):
        balls = []
        if not detections:
            return balls

        boxes, class_ids, scores = detections
        for index in range(len(boxes)):
            if int(class_ids[index]) != 0:
                continue

            x, y, width, height = map(lambda value: int(round(value, 0)), boxes[index])
            width = max(0, width)
            height = max(0, height)
            if width <= 1 or height <= 1:
                continue

            balls.append(
                {
                    "x1": x,
                    "y1": y,
                    "x2": x + width,
                    "y2": y + height,
                    "cx": x + width // 2,
                    "cy": y + height // 2,
                    "width": width,
                    "height": height,
                    "confidence": float(scores[index]),
                }
            )

        balls.sort(key=lambda ball: ball["confidence"], reverse=True)
        return balls

    def draw_result(self, pipeline, balls, selected_ball, stable):
        pipeline.osd_img.clear()
        for ball in balls:
            color = self.colors[0]
            thickness = 3
            if ball is selected_ball:
                color = (255, 0, 255, 0) if stable else (255, 255, 165, 0)
                thickness = 5

            pipeline.osd_img.draw_rectangle(
                ball["x1"],
                ball["y1"],
                ball["width"],
                ball["height"],
                color=color,
                thickness=thickness,
            )
            label = "steel_ball %.2f" % ball["confidence"]
            pipeline.osd_img.draw_string_advanced(
                ball["x1"],
                max(0, ball["y1"] - 32),
                24,
                label,
                color=color,
            )


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
    header = bytes(
        [SERIAL_SOF0, SERIAL_SOF1, msg_id & 0xFF, seq & 0xFF, len(payload) & 0xFF]
    )
    return header + payload + bytes([crc8(header[2:] + payload)])


def ball_payload(ball, stable, center_x):
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
        int(ball["cx"] - center_x),
        min(100, int(ball["confidence"] * 100)),
        flags,
    )


def multi_ball_payload(balls):
    selected = balls[:MAX_MULTI_BALLS]
    payload = bytearray([len(selected)])
    for ball in selected:
        radius = max(ball["width"], ball["height"]) // 2
        payload += struct.pack(
            "<hhHB",
            int(ball["cx"]),
            int(ball["cy"]),
            int(radius),
            min(100, int(ball["confidence"] * 100)),
        )
    return bytes(payload)


def setup_uart():
    fpioa = FPIOA()
    fpioa.set_function(K230_UART_TX_GPIO, FPIOA.UART2_TXD)
    fpioa.set_function(K230_UART_RX_GPIO, FPIOA.UART2_RXD)
    return UART(
        UART_ID,
        baudrate=UART_BAUD,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )


def choose_best_detection(balls, center_x, image_width, image_height):
    best = None
    best_score = -1.0
    for ball in balls:
        center_penalty = abs(ball["cx"] - center_x) / max(1, center_x)
        lower_bonus = ball["cy"] / max(1, image_height)
        area_bonus = (ball["width"] * ball["height"]) / max(
            1, image_width * image_height
        )
        score = (
            ball["confidence"]
            + lower_bonus * 0.08
            + area_bonus * 0.15
            - center_penalty * 0.04
        )
        if score > best_score:
            best_score = score
            best = ball
    return best


def main():
    pipeline = None
    detector = None
    uart = None
    seq = 0
    last_ball = None
    stable_count = 0

    try:
        uart = setup_uart()
        pipeline = PipeLine(
            rgb888p_size=RGB888P_SIZE,
            display_mode=DISPLAY_MODE,
            display_size=DISPLAY_SIZE,
        )
        pipeline.create()
        display_size = pipeline.get_display_size()
        center_x = display_size[0] // 2

        detector = SteelBallDetectionApp(
            MODEL_PATH,
            model_input_size=MODEL_INPUT_SIZE,
            rgb888p_size=RGB888P_SIZE,
            display_size=display_size,
            confidence_threshold=CONF_THRESHOLD,
            nms_threshold=NMS_THRESHOLD,
            max_boxes_num=MAX_BOXES_NUM,
            debug_mode=0,
        )
        detector.config_preprocess()

        print("Steel-ball detector ready")
        print("Model:", MODEL_PATH)
        print("UART2: TX GPIO%d, RX GPIO%d, %d baud" % (
            K230_UART_TX_GPIO,
            K230_UART_RX_GPIO,
            UART_BAUD,
        ))

        while True:
            frame = pipeline.get_frame()
            detections = detector.run(frame)
            balls = detector.to_ball_list(detections)
            ball = choose_best_detection(
                balls,
                center_x,
                display_size[0],
                display_size[1],
            )

            if ball is None:
                stable_count = 0
            elif last_ball is not None:
                dx = ball["cx"] - last_ball["cx"]
                dy = ball["cy"] - last_ball["cy"]
                if dx * dx + dy * dy <= TRACK_GATE_PIXELS * TRACK_GATE_PIXELS:
                    stable_count += 1
                else:
                    stable_count = 1
            else:
                stable_count = 1

            stable = stable_count >= STABLE_FRAMES
            uart.write(pack_frame(MSG_VISION_BALL, seq, ball_payload(ball, stable, center_x)))
            seq = (seq + 1) & 0xFF
            uart.write(pack_frame(MSG_VISION_MULTI_BALL, seq, multi_ball_payload(balls)))
            seq = (seq + 1) & 0xFF

            detector.draw_result(pipeline, balls, ball, stable)
            pipeline.show_image()
            last_ball = ball
            gc.collect()
    except KeyboardInterrupt:
        print("Stopped by user")
    except Exception as error:
        sys.print_exception(error)
    finally:
        if detector is not None:
            detector.deinit()
        if pipeline is not None:
            pipeline.destroy()
        if uart is not None:
            uart.deinit()
        gc.collect()


if __name__ == "__main__":
    main()
