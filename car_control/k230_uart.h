#ifndef K230_UART_H
#define K230_UART_H

#include <stdbool.h>
#include <stdint.h>

#define K230_BALL_FLAG_DETECTED 0x01U
#define K230_BALL_FLAG_STABLE   0x02U
#define K230_BALL_FLAG_CLOSE    0x04U

typedef struct {
    int16_t x;
    int16_t y;
    uint16_t radius;
    int16_t offset_x;
    uint8_t confidence;
    uint8_t flags;
} K230_Ball_t;

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
} K230_UartStatus_t;

void K230Uart_Init(void);
void K230Uart_Poll(void);
const K230_UartStatus_t *K230Uart_GetStatus(void);
bool K230Uart_GetBall(K230_Ball_t *ball);

#endif
