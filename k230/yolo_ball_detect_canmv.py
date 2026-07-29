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
from libs.Utils import ScopedTiming, get_colors, letterbox_pad_param


# The camera frame is letterboxed to 416x416 by AI2D before inference.
RGB888P_SIZE = [640, 480]
MODEL_INPUT_SIZE = [416, 416]
DISPLAY_MODE = "virt"  # Use "hdmi" when an HDMI monitor is connected.
DISPLAY_SIZE = [800, 480]

MODEL_PATH = "/sdcard/examples/kmodel/steel_ball_yolo11n_hardcase_ft.kmodel"
LABELS = ["steel_ball"]
CONF_THRESHOLD = 0.22
NMS_THRESHOLD = 0.55
MAX_BOXES_NUM = 30

MAX_MULTI_BALLS = 4
CLOSE_BOX_HEIGHT = 120
BALL_MIN_ASPECT_PERCENT = 55
STABLE_FRAMES = 4
TRACK_CONFIRM_FRAMES = 3
TRACK_CONFIRM_CONFIDENCE = 0.30
TRACK_MAX_MISSES = 2
TRACK_MATCH_PIXELS = 70
TRACK_SMOOTH_ALPHA = 0.45
MAX_TRACKS = 8

UART_BAUD = 115200
UART_ID = 2
K230_UART_TX_GPIO = 5
K230_UART_RX_GPIO = 6
DEBUG_PRINT = True
DEBUG_PRINT_INTERVAL = 15

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
        self.rgb888p_size = [((rgb888p_size[0] + 15) // 16) * 16, rgb888p_size[1]]
        self.display_size = [((display_size[0] + 15) // 16) * 16, display_size[1]]
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
            short_side = min(width, height)
            long_side = max(width, height)
            if short_side * 100 < long_side * BALL_MIN_ASPECT_PERCENT:
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
            if not ball["observed"]:
                color = (255, 128, 128, 128)
                thickness = 2
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
            label = "T%d %.2f" % (ball["track_id"], ball["confidence"])
            pipeline.osd_img.draw_string_advanced(
                ball["x1"],
                max(0, ball["y1"] - 32),
                24,
                label,
                color=color,
            )


class MultiBallTracker:
    def __init__(self):
        self.tracks = []
        self.next_track_id = 1

    def _create_track(self, detection):
        track = detection.copy()
        track["track_id"] = self.next_track_id
        track["hits"] = 1
        track["streak"] = 1
        track["missed"] = 0
        track["observed"] = True
        track["confirmed"] = False
        self.next_track_id += 1
        if self.next_track_id > 255:
            self.next_track_id = 1
        return track

    def _update_track(self, track, detection):
        alpha = TRACK_SMOOTH_ALPHA
        for field in ("x1", "y1", "width", "height"):
            track[field] = int(
                track[field] * (1.0 - alpha) + detection[field] * alpha + 0.5
            )
        track["x2"] = track["x1"] + track["width"]
        track["y2"] = track["y1"] + track["height"]
        track["cx"] = track["x1"] + track["width"] // 2
        track["cy"] = track["y1"] + track["height"] // 2
        track["confidence"] = (
            track["confidence"] * (1.0 - alpha)
            + detection["confidence"] * alpha
        )
        track["hits"] += 1
        track["streak"] += 1
        track["missed"] = 0
        track["observed"] = True
        if (
            track["streak"] >= TRACK_CONFIRM_FRAMES
            and track["confidence"] >= TRACK_CONFIRM_CONFIDENCE
        ):
            track["confirmed"] = True

    def update(self, detections):
        detections = detections[:MAX_TRACKS]
        pairs = []
        for track_index in range(len(self.tracks)):
            track = self.tracks[track_index]
            for detection_index in range(len(detections)):
                detection = detections[detection_index]
                dx = track["cx"] - detection["cx"]
                dy = track["cy"] - detection["cy"]
                gate = max(
                    TRACK_MATCH_PIXELS,
                    (track["width"] + track["height"]
                     + detection["width"] + detection["height"]) // 4,
                )
                distance_squared = dx * dx + dy * dy
                if distance_squared <= gate * gate:
                    pairs.append((distance_squared, track_index, detection_index))

        pairs.sort(key=lambda pair: pair[0])
        matched_tracks = [False] * len(self.tracks)
        matched_detections = [False] * len(detections)

        for _, track_index, detection_index in pairs:
            if matched_tracks[track_index] or matched_detections[detection_index]:
                continue
            self._update_track(
                self.tracks[track_index],
                detections[detection_index],
            )
            matched_tracks[track_index] = True
            matched_detections[detection_index] = True

        retained_tracks = []
        for track_index in range(len(self.tracks)):
            track = self.tracks[track_index]
            if not matched_tracks[track_index]:
                track["missed"] += 1
                track["streak"] = 0
                track["observed"] = False
                track["confidence"] *= 0.85
            if track["missed"] <= TRACK_MAX_MISSES:
                retained_tracks.append(track)
        self.tracks = retained_tracks

        for detection_index in range(len(detections)):
            if matched_detections[detection_index]:
                continue
            if len(self.tracks) >= MAX_TRACKS:
                break
            self.tracks.append(self._create_track(detections[detection_index]))

        confirmed = []
        for track in self.tracks:
            if track["confirmed"]:
                confirmed.append(track)
        confirmed.sort(key=lambda track: track["confidence"], reverse=True)
        return confirmed


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


def debug_print_detection(frame_index, ball, balls, stable, center_x):
    if not DEBUG_PRINT or frame_index % DEBUG_PRINT_INTERVAL != 0:
        return

    if ball is None:
        print("VISION frame=%d main=none count=%d" % (
            frame_index,
            min(len(balls), MAX_MULTI_BALLS),
        ))
        return

    flags = BALL_FLAG_DETECTED
    if stable:
        flags |= BALL_FLAG_STABLE
    if ball["height"] >= CLOSE_BOX_HEIGHT:
        flags |= BALL_FLAG_CLOSE

    print(
        "VISION frame=%d main=T%d x=%d y=%d r=%d dx=%d conf=%d flags=0x%02X count=%d"
        % (
            frame_index,
            ball["track_id"],
            int(ball["cx"]),
            int(ball["cy"]),
            max(ball["width"], ball["height"]) // 2,
            int(ball["cx"] - center_x),
            min(100, int(ball["confidence"] * 100)),
            flags,
            min(len(balls), MAX_MULTI_BALLS),
        )
    )


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


def choose_best_detection(
    balls,
    center_x,
    image_width,
    image_height,
    preferred_track_id=None,
):
    if preferred_track_id is not None:
        for ball in balls:
            if ball["track_id"] == preferred_track_id:
                return ball

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
            - ball["missed"] * 0.08
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
    frame_index = 0
    tracker = MultiBallTracker()
    primary_track_id = None

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
            raw_balls = detector.to_ball_list(detections)
            balls = tracker.update(raw_balls)
            ball = choose_best_detection(
                balls,
                center_x,
                display_size[0],
                display_size[1],
                primary_track_id,
            )

            if ball is None:
                primary_track_id = None
            else:
                primary_track_id = ball["track_id"]

            stable = (
                ball is not None
                and ball["observed"]
                and ball["streak"] >= STABLE_FRAMES
            )
            debug_print_detection(frame_index, ball, balls, stable, center_x)
            uart.write(pack_frame(MSG_VISION_BALL, seq, ball_payload(ball, stable, center_x)))
            seq = (seq + 1) & 0xFF
            uart.write(pack_frame(MSG_VISION_MULTI_BALL, seq, multi_ball_payload(balls)))
            seq = (seq + 1) & 0xFF
            frame_index += 1

            detector.draw_result(pipeline, balls, ball, stable)
            pipeline.show_image()
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
