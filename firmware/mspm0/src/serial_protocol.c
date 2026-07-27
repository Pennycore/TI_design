#include "serial_protocol.h"

#include <string.h>

enum {
    PARSER_WAIT_SOF0 = 0,
    PARSER_WAIT_SOF1,
    PARSER_MSG_ID,
    PARSER_SEQ,
    PARSER_LEN,
    PARSER_PAYLOAD,
    PARSER_CRC,
};

static uint8_t crc8_update(uint8_t crc, uint8_t data)
{
    crc ^= data;
    for (uint8_t i = 0U; i < 8U; ++i) {
        if ((crc & 0x80U) != 0U) {
            crc = (uint8_t)((crc << 1U) ^ 0x07U);
        } else {
            crc = (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static void write_u16_le(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)(value & 0xFFU);
    out[1] = (uint8_t)(value >> 8U);
}

static void write_i16_le(uint8_t *out, int16_t value)
{
    write_u16_le(out, (uint16_t)value);
}

static void write_u32_le(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)(value & 0xFFU);
    out[1] = (uint8_t)((value >> 8U) & 0xFFU);
    out[2] = (uint8_t)((value >> 16U) & 0xFFU);
    out[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

uint8_t serial_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0U;
    for (size_t i = 0U; i < len; ++i) {
        crc = crc8_update(crc, data[i]);
    }
    return crc;
}

void serial_parser_init(serial_parser_t *parser)
{
    memset(parser, 0, sizeof(*parser));
    parser->state = PARSER_WAIT_SOF0;
}

bool serial_parser_push(serial_parser_t *parser, uint8_t byte, serial_frame_t *out_frame)
{
    switch (parser->state) {
    case PARSER_WAIT_SOF0:
        if (byte == SERIAL_SOF0) {
            parser->state = PARSER_WAIT_SOF1;
        }
        break;

    case PARSER_WAIT_SOF1:
        parser->state = (byte == SERIAL_SOF1) ? PARSER_MSG_ID : PARSER_WAIT_SOF0;
        break;

    case PARSER_MSG_ID:
        parser->msg_id = byte;
        parser->crc = crc8_update(0U, byte);
        parser->state = PARSER_SEQ;
        break;

    case PARSER_SEQ:
        parser->seq = byte;
        parser->crc = crc8_update(parser->crc, byte);
        parser->state = PARSER_LEN;
        break;

    case PARSER_LEN:
        parser->len = byte;
        parser->index = 0U;
        parser->crc = crc8_update(parser->crc, byte);
        if (parser->len > SERIAL_MAX_PAYLOAD) {
            serial_parser_init(parser);
        } else {
            parser->state = (parser->len == 0U) ? PARSER_CRC : PARSER_PAYLOAD;
        }
        break;

    case PARSER_PAYLOAD:
        parser->payload[parser->index++] = byte;
        parser->crc = crc8_update(parser->crc, byte);
        if (parser->index >= parser->len) {
            parser->state = PARSER_CRC;
        }
        break;

    case PARSER_CRC:
        if (byte == parser->crc) {
            out_frame->msg_id = parser->msg_id;
            out_frame->seq = parser->seq;
            out_frame->len = parser->len;
            memcpy(out_frame->payload, parser->payload, parser->len);
            serial_parser_init(parser);
            return true;
        }
        serial_parser_init(parser);
        break;

    default:
        serial_parser_init(parser);
        break;
    }

    return false;
}

size_t serial_encode(uint8_t msg_id,
                     uint8_t seq,
                     const uint8_t *payload,
                     uint8_t payload_len,
                     uint8_t *out,
                     size_t out_capacity)
{
    size_t frame_len = (size_t)payload_len + 6U;

    if ((payload_len > SERIAL_MAX_PAYLOAD) || (out_capacity < frame_len)) {
        return 0U;
    }

    out[0] = SERIAL_SOF0;
    out[1] = SERIAL_SOF1;
    out[2] = msg_id;
    out[3] = seq;
    out[4] = payload_len;
    if (payload_len > 0U) {
        memcpy(&out[5], payload, payload_len);
    }
    out[5U + payload_len] = serial_crc8(&out[2], (size_t)payload_len + 3U);
    return frame_len;
}

bool serial_decode_vision_ball(const serial_frame_t *frame, vision_ball_t *ball)
{
    if ((frame->msg_id != MSG_VISION_BALL) || (frame->len != 10U)) {
        return false;
    }

    ball->x = read_i16_le(&frame->payload[0]);
    ball->y = read_i16_le(&frame->payload[2]);
    ball->radius = read_u16_le(&frame->payload[4]);
    ball->offset_x = read_i16_le(&frame->payload[6]);
    ball->confidence = frame->payload[8];
    ball->flags = frame->payload[9];
    return true;
}

uint8_t serial_encode_mcu_telemetry_payload(const mcu_telemetry_t *telemetry, uint8_t out[12])
{
    write_u32_le(&out[0], telemetry->time_ms);
    write_i16_le(&out[4], telemetry->line_pos);
    write_i16_le(&out[6], telemetry->left_pwm);
    write_i16_le(&out[8], telemetry->right_pwm);
    out[10] = telemetry->state;
    out[11] = telemetry->gray_bits;
    return 12U;
}
