#include "k230_uart.h"

#include <stddef.h>

#include "ti_msp_dl_config.h"

#define SERIAL_SOF0 0xA5U
#define SERIAL_SOF1 0x5AU
#define MSG_VISION_BALL 0x10U
#define MSG_VISION_MULTI_BALL 0x11U
#define MSG_ROD_BALL_POSITION 0x12U
#define SERIAL_MAX_PAYLOAD 32U

typedef enum {
    PARSER_WAIT_SOF0 = 0,
    PARSER_WAIT_SOF1,
    PARSER_MSG_ID,
    PARSER_SEQ,
    PARSER_LEN,
    PARSER_PAYLOAD,
    PARSER_CRC,
} ParserState_t;

typedef struct {
    ParserState_t state;
    uint8_t msg_id;
    uint8_t seq;
    uint8_t len;
    uint8_t index;
    uint8_t crc;
    uint8_t payload[SERIAL_MAX_PAYLOAD];
} Parser_t;

volatile K230_UartStatus_t g_k230_uart_status;

static Parser_t g_parser;

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

static void parser_reset(void)
{
    g_parser.state = PARSER_WAIT_SOF0;
    g_parser.msg_id = 0U;
    g_parser.seq = 0U;
    g_parser.len = 0U;
    g_parser.index = 0U;
    g_parser.crc = 0U;
}

static void decode_ball_payload(const uint8_t payload[10])
{
    K230_Ball_t ball;

    ball.x = read_i16_le(&payload[0]);
    ball.y = read_i16_le(&payload[2]);
    ball.radius = read_u16_le(&payload[4]);
    ball.offset_x = read_i16_le(&payload[6]);
    ball.confidence = payload[8];
    ball.flags = payload[9];

    g_k230_uart_status.ball = ball;
    g_k230_uart_status.ball_detected =
        (uint8_t)((ball.flags & K230_BALL_FLAG_DETECTED) != 0U);
    g_k230_uart_status.ball_stable =
        (uint8_t)((ball.flags & K230_BALL_FLAG_STABLE) != 0U);
    g_k230_uart_status.ball_close =
        (uint8_t)((ball.flags & K230_BALL_FLAG_CLOSE) != 0U);
}

static void decode_rod_ball_payload(const uint8_t payload[8])
{
    K230_RodBall_t rod_ball;

    rod_ball.position_mm = read_i16_le(&payload[0]);
    rod_ball.velocity_mm_s = read_i16_le(&payload[2]);
    rod_ball.raw_x = read_u16_le(&payload[4]);
    rod_ball.confidence = payload[6];
    rod_ball.flags = payload[7];

    g_k230_uart_status.rod_ball = rod_ball;
    g_k230_uart_status.rod_ball_detected =
        (uint8_t)((rod_ball.flags & K230_ROD_FLAG_DETECTED) != 0U);
    g_k230_uart_status.rod_ball_stable =
        (uint8_t)((rod_ball.flags & K230_ROD_FLAG_STABLE) != 0U);
    g_k230_uart_status.rod_ball_on_target =
        (uint8_t)((rod_ball.flags & K230_ROD_FLAG_ON_TARGET) != 0U);
    g_k230_uart_status.rod_ball_predicted =
        (uint8_t)((rod_ball.flags & K230_ROD_FLAG_PREDICTED) != 0U);
    g_k230_uart_status.rod_ball_valid =
        (uint8_t)((g_k230_uart_status.rod_ball_detected != 0U) &&
                  (g_k230_uart_status.rod_ball_stable != 0U));
}

static void handle_frame(void)
{
    g_k230_uart_status.frame_count++;
    g_k230_uart_status.last_msg_id = g_parser.msg_id;
    g_k230_uart_status.last_seq = g_parser.seq;

    if ((g_parser.msg_id == MSG_VISION_BALL) && (g_parser.len == 10U)) {
        decode_ball_payload(g_parser.payload);
    } else if ((g_parser.msg_id == MSG_ROD_BALL_POSITION) &&
               (g_parser.len == 8U)) {
        decode_rod_ball_payload(g_parser.payload);
    }
}

static void parser_push(uint8_t byte)
{
    switch (g_parser.state) {
    case PARSER_WAIT_SOF0:
        if (byte == SERIAL_SOF0) {
            g_parser.state = PARSER_WAIT_SOF1;
        }
        break;

    case PARSER_WAIT_SOF1:
        g_parser.state = (byte == SERIAL_SOF1) ? PARSER_MSG_ID : PARSER_WAIT_SOF0;
        break;

    case PARSER_MSG_ID:
        g_parser.msg_id = byte;
        g_parser.crc = crc8_update(0U, byte);
        g_parser.state = PARSER_SEQ;
        break;

    case PARSER_SEQ:
        g_parser.seq = byte;
        g_parser.crc = crc8_update(g_parser.crc, byte);
        g_parser.state = PARSER_LEN;
        break;

    case PARSER_LEN:
        g_parser.len = byte;
        g_parser.index = 0U;
        g_parser.crc = crc8_update(g_parser.crc, byte);
        if (g_parser.len > SERIAL_MAX_PAYLOAD) {
            g_k230_uart_status.bad_length_count++;
            parser_reset();
        } else {
            g_parser.state = (g_parser.len == 0U) ? PARSER_CRC : PARSER_PAYLOAD;
        }
        break;

    case PARSER_PAYLOAD:
        g_parser.payload[g_parser.index++] = byte;
        g_parser.crc = crc8_update(g_parser.crc, byte);
        if (g_parser.index >= g_parser.len) {
            g_parser.state = PARSER_CRC;
        }
        break;

    case PARSER_CRC:
        if (byte == g_parser.crc) {
            handle_frame();
        } else {
            g_k230_uart_status.crc_error_count++;
        }
        parser_reset();
        break;

    default:
        parser_reset();
        break;
    }
}

void K230Uart_Init(void)
{
    parser_reset();
}

void K230Uart_Poll(void)
{
    uint8_t byte;

    while (DL_UART_Main_receiveDataCheck(UART_K230_INST, &byte)) {
        g_k230_uart_status.byte_count++;
        parser_push(byte);
    }
}

const K230_UartStatus_t *K230Uart_GetStatus(void)
{
    return (const K230_UartStatus_t *)&g_k230_uart_status;
}

bool K230Uart_GetBall(K230_Ball_t *ball)
{
    if (ball == NULL) {
        return false;
    }

    *ball = g_k230_uart_status.ball;
    return g_k230_uart_status.ball_detected != 0U;
}

bool K230Uart_GetRodBall(K230_RodBall_t *rod_ball)
{
    if (rod_ball == NULL) {
        return false;
    }

    *rod_ball = g_k230_uart_status.rod_ball;
    return g_k230_uart_status.rod_ball_valid != 0U;
}
