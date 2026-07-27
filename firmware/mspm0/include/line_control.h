#ifndef LINE_CONTROL_H
#define LINE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "board_config.h"

typedef struct {
    bool valid;
    int16_t position;
    uint8_t active_mask;
    uint8_t confidence;
} line_estimate_t;

typedef struct {
    int16_t kp;
    int16_t ki;
    int16_t kd;
    int32_t integral;
    int16_t previous_error;
    int32_t integral_limit;
} line_pid_t;

typedef struct {
    int16_t left_pwm;
    int16_t right_pwm;
} drive_command_t;

void line_pid_init(line_pid_t *pid);
line_estimate_t line_estimate_from_analog(const uint8_t analog[ROBOT_GRAY_SENSOR_COUNT], bool black_line);
line_estimate_t line_estimate_from_digital(uint8_t digital_bits, bool black_line);
drive_command_t line_pid_update(line_pid_t *pid, const line_estimate_t *estimate, int16_t base_pwm);

#endif
