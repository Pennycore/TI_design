#ifndef K230_UART_H
#define K230_UART_H

#include <stdbool.h>
#include <stdint.h>

#define K230_BALL_FLAG_DETECTED 0x01U
#define K230_BALL_FLAG_STABLE   0x02U
#define K230_BALL_FLAG_CLOSE    0x04U

#define K230_ROD_FLAG_DETECTED  0x01U
#define K230_ROD_FLAG_STABLE    0x02U
#define K230_ROD_FLAG_ON_TARGET 0x04U
#define K230_ROD_FLAG_PREDICTED 0x08U

typedef struct {
    int16_t x;
    int16_t y;
    uint16_t radius;
    int16_t offset_x;
    uint8_t confidence;
    uint8_t flags;
} K230_Ball_t;

typedef struct {
    int16_t position_mm;
    int16_t velocity_mm_s;
    uint16_t raw_x;
    uint8_t confidence;
    uint8_t flags;
} K230_RodBall_t;

typedef struct {
    volatile uint32_t byte_count;
    volatile uint32_t frame_count;
    volatile uint32_t crc_error_count;
    volatile uint32_t bad_length_count;
    volatile uint8_t last_msg_id;
    volatile uint8_t last_seq;
    volatile uint8_t ball_detected;
    volatile uint8_t ball_stable;
    volatile uint8_t ball_close;
    volatile K230_Ball_t ball;
    volatile uint8_t rod_ball_detected;
    volatile uint8_t rod_ball_stable;
    volatile uint8_t rod_ball_on_target;
    volatile uint8_t rod_ball_predicted;
    volatile uint8_t rod_ball_valid;
    volatile K230_RodBall_t rod_ball;
} K230_UartStatus_t;

void K230Uart_Init(void);
void K230Uart_Poll(void);
const K230_UartStatus_t *K230Uart_GetStatus(void);
bool K230Uart_GetBall(K230_Ball_t *ball);
bool K230Uart_GetRodBall(K230_RodBall_t *rod_ball);

#endif
