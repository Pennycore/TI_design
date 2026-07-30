#!/usr/bin/env python3
import argparse
import struct
import sys


SERIAL_SOF0 = 0xA5
SERIAL_SOF1 = 0x5A
MSG_VISION_BALL = 0x10
MSG_VISION_MULTI_BALL = 0x11
MSG_ROD_BALL_POSITION = 0x12
MSG_MCU_TELEMETRY = 0x20


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


class Parser:
    def __init__(self):
        self.reset()

    def reset(self):
        self.state = 0
        self.msg_id = 0
        self.seq = 0
        self.length = 0
        self.payload = bytearray()

    def push(self, byte):
        if self.state == 0:
            if byte == SERIAL_SOF0:
                self.state = 1
            return None

        if self.state == 1:
            self.state = 2 if byte == SERIAL_SOF1 else 0
            return None

        if self.state == 2:
            self.msg_id = byte
            self.state = 3
            return None

        if self.state == 3:
            self.seq = byte
            self.state = 4
            return None

        if self.state == 4:
            self.length = byte
            self.payload = bytearray()
            if self.length > 32:
                self.reset()
            else:
                self.state = 6 if self.length == 0 else 5
            return None

        if self.state == 5:
            self.payload.append(byte)
            if len(self.payload) >= self.length:
                self.state = 6
            return None

        if self.state == 6:
            body = bytes([self.msg_id, self.seq, self.length]) + bytes(self.payload)
            if crc8(body) == byte:
                frame = (self.msg_id, self.seq, bytes(self.payload))
                self.reset()
                return frame
            self.reset()
            return None

        self.reset()
        return None


def describe_frame(frame):
    msg_id, seq, payload = frame
    if msg_id == MSG_VISION_BALL and len(payload) == 10:
        x, y, radius, offset_x, confidence, flags = struct.unpack("<hhHhBB", payload)
        return (
            f"VISION seq={seq:03d} x={x:4d} y={y:4d} r={radius:3d} "
            f"dx={offset_x:4d} conf={confidence:3d} flags=0x{flags:02X}"
        )

    if msg_id == MSG_VISION_MULTI_BALL and len(payload) >= 1:
        count = payload[0]
        balls = []
        offset = 1
        for index in range(min(count, 4)):
            if offset + 7 > len(payload):
                break
            x, y, radius, confidence = struct.unpack("<hhHB", payload[offset:offset + 7])
            balls.append(
                f"b{index}=({x},{y},r={radius},c={confidence})"
            )
            offset += 7
        return f"MULTI  seq={seq:03d} count={count} " + " ".join(balls)

    if msg_id == MSG_ROD_BALL_POSITION and len(payload) == 8:
        position_mm, velocity_mm_s, raw_x, confidence, flags = struct.unpack(
            "<hhHBB", payload
        )
        return (
            f"ROD seq={seq:03d} pos={position_mm:4d}mm "
            f"v={velocity_mm_s:5d}mm/s x={raw_x:3d} "
            f"conf={confidence:3d} flags=0x{flags:02X}"
        )

    if msg_id == MSG_MCU_TELEMETRY and len(payload) == 12:
        time_ms, line_pos, left_pwm, right_pwm, state, gray_bits = struct.unpack("<IhhhBB", payload)
        return (
            f"MCU seq={seq:03d} t={time_ms:8d} line={line_pos:5d} "
            f"L={left_pwm:5d} R={right_pwm:5d} state={state} gray=0b{gray_bits:08b}"
        )

    return f"MSG 0x{msg_id:02X} seq={seq:03d} len={len(payload)} payload={payload.hex()}"


def main():
    parser = argparse.ArgumentParser(description="Monitor MSPM0/K230 robot UART frames.")
    parser.add_argument("port", help="Serial port, for example COM6")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    try:
        import serial
    except ImportError:
        print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
        return 2

    frame_parser = Parser()
    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        print(f"Listening on {args.port} at {args.baud} baud")
        while True:
            data = ser.read(256)
            for byte in data:
                frame = frame_parser.push(byte)
                if frame is not None:
                    print(describe_frame(frame))


if __name__ == "__main__":
    raise SystemExit(main())
