#ifndef ROBOT_APP_H
#define ROBOT_APP_H

#include <stdint.h>

#include "gw_gray.h"
#include "line_control.h"
#include "serial_protocol.h"

typedef enum {
    ROBOT_STATE_BOOT = 0,
    ROBOT_STATE_LINE_FOLLOW = 1,
    ROBOT_STATE_BALL_ALIGN = 2,
    ROBOT_STATE_STOPPED = 3,
    ROBOT_STATE_SENSOR_FAULT = 4,
} robot_state_t;

typedef struct {
    robot_state_t state;
    gw_gray_t gray;
    line_pid_t line_pid;
    serial_parser_t serial_parser;
    vision_ball_t last_ball;
    uint32_t last_ball_ms;
    uint32_t last_tick_ms;
    uint32_t last_telemetry_ms;
    line_estimate_t last_line;
    drive_command_t last_drive;
    uint8_t telemetry_seq;
    uint8_t gray_bits;
} robot_app_t;

void robot_app_init(robot_app_t *app);
void robot_app_poll_uart(robot_app_t *app);
void robot_app_tick(robot_app_t *app);

#endif
