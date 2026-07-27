#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SERIAL_SOF0 0xA5U
#define SERIAL_SOF1 0x5AU
#define SERIAL_MAX_PAYLOAD 32U

typedef enum {
    MSG_VISION_BALL = 0x10,
    MSG_MCU_TELEMETRY = 0x20,
} serial_msg_id_t;

typedef struct {
    uint8_t msg_id;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[SERIAL_MAX_PAYLOAD];
} serial_frame_t;

typedef struct {
    uint8_t state;
    uint8_t msg_id;
    uint8_t seq;
    uint8_t len;
    uint8_t index;
    uint8_t crc;
    uint8_t payload[SERIAL_MAX_PAYLOAD];
} serial_parser_t;

typedef struct {
    int16_t x;
    int16_t y;
    uint16_t radius;
    int16_t offset_x;
    uint8_t confidence;
    uint8_t flags;
} vision_ball_t;

typedef struct {
    uint32_t time_ms;
    int16_t line_pos;
    int16_t left_pwm;
    int16_t right_pwm;
    uint8_t state;
    uint8_t gray_bits;
} mcu_telemetry_t;

void serial_parser_init(serial_parser_t *parser);
bool serial_parser_push(serial_parser_t *parser, uint8_t byte, serial_frame_t *out_frame);
size_t serial_encode(uint8_t msg_id,
                     uint8_t seq,
                     const uint8_t *payload,
                     uint8_t payload_len,
                     uint8_t *out,
                     size_t out_capacity);

uint8_t serial_crc8(const uint8_t *data, size_t len);
bool serial_decode_vision_ball(const serial_frame_t *frame, vision_ball_t *ball);
uint8_t serial_encode_mcu_telemetry_payload(const mcu_telemetry_t *telemetry, uint8_t out[12]);

#endif
