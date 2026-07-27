#include <assert.h>
#include <stdint.h>

#include "line_control.h"
#include "serial_protocol.h"

static void test_line_estimate(void)
{
    uint8_t analog[ROBOT_GRAY_SENSOR_COUNT] = {255, 255, 255, 20, 10, 255, 255, 255};
    line_estimate_t estimate = line_estimate_from_analog(analog, true);

    assert(estimate.valid);
    assert(estimate.position > -700);
    assert(estimate.position < 700);
    assert((estimate.active_mask & 0x18U) == 0x18U);
}

static void test_protocol_roundtrip(void)
{
    uint8_t payload[10] = {
        160, 0, 120, 0, 20, 0, 0, 0, 80, 0x03,
    };
    uint8_t frame_bytes[32];
    serial_parser_t parser;
    serial_frame_t frame;
    vision_ball_t ball;

    size_t len = serial_encode(MSG_VISION_BALL, 7U, payload, sizeof(payload), frame_bytes, sizeof(frame_bytes));
    assert(len == 16U);

    serial_parser_init(&parser);
    for (size_t i = 0U; i < len; ++i) {
        if (i + 1U < len) {
            assert(!serial_parser_push(&parser, frame_bytes[i], &frame));
        } else {
            assert(serial_parser_push(&parser, frame_bytes[i], &frame));
        }
    }

    assert(serial_decode_vision_ball(&frame, &ball));
    assert(ball.x == 160);
    assert(ball.y == 120);
    assert(ball.radius == 20);
    assert(ball.confidence == 80);
    assert(ball.flags == 0x03);
}

int main(void)
{
    test_line_estimate();
    test_protocol_roundtrip();
    return 0;
}
