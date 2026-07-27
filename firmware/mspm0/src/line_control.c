#include "line_control.h"

#include <stddef.h>

static const int16_t k_sensor_weights[ROBOT_GRAY_SENSOR_COUNT] = {
    -3500, -2500, -1500, -500, 500, 1500, 2500, 3500,
};

static int16_t clamp_i16(int32_t value, int16_t min_value, int16_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return (int16_t)value;
}

void line_pid_init(line_pid_t *pid)
{
    pid->kp = ROBOT_LINE_KP;
    pid->ki = ROBOT_LINE_KI;
    pid->kd = ROBOT_LINE_KD;
    pid->integral = 0;
    pid->previous_error = 0;
    pid->integral_limit = ROBOT_LINE_INTEGRAL_LIMIT;
}

line_estimate_t line_estimate_from_analog(const uint8_t analog[ROBOT_GRAY_SENSOR_COUNT], bool black_line)
{
    line_estimate_t estimate = {0};
    uint32_t sum = 0U;
    int32_t weighted = 0;

    for (size_t i = 0U; i < ROBOT_GRAY_SENSOR_COUNT; ++i) {
        uint16_t signal = black_line ? (uint16_t)(255U - analog[i]) : analog[i];

        if (signal > ROBOT_LINE_SIGNAL_THRESHOLD) {
            estimate.active_mask |= (uint8_t)(1U << i);
        }

        sum += signal;
        weighted += (int32_t)signal * k_sensor_weights[i];
    }

    if (sum < ROBOT_LINE_MIN_SUM) {
        estimate.valid = false;
        estimate.position = 0;
        estimate.confidence = 0U;
        return estimate;
    }

    estimate.valid = true;
    estimate.position = clamp_i16(weighted / (int32_t)sum, -3500, 3500);
    estimate.confidence = (uint8_t)((sum > 1020U) ? 100U : ((sum * 100U) / 1020U));
    return estimate;
}

line_estimate_t line_estimate_from_digital(uint8_t digital_bits, bool black_line)
{
    line_estimate_t estimate = {0};
    uint32_t sum = 0U;
    int32_t weighted = 0;
    uint8_t active_count = 0U;

    for (size_t i = 0U; i < ROBOT_GRAY_SENSOR_COUNT; ++i) {
        bool white = ((digital_bits >> i) & 0x01U) != 0U;
        bool active = black_line ? !white : white;

        if (!active) {
            continue;
        }

        estimate.active_mask |= (uint8_t)(1U << i);
        active_count++;
        sum += 255U;
        weighted += 255 * k_sensor_weights[i];
    }

    if (sum == 0U) {
        estimate.valid = false;
        return estimate;
    }

    estimate.valid = true;
    estimate.position = clamp_i16(weighted / (int32_t)sum, -3500, 3500);
    estimate.confidence = (uint8_t)((active_count * 100U) / ROBOT_GRAY_SENSOR_COUNT);
    return estimate;
}

drive_command_t line_pid_update(line_pid_t *pid, const line_estimate_t *estimate, int16_t base_pwm)
{
    drive_command_t command = {0};

    if (!estimate->valid) {
        pid->integral = 0;
        command.left_pwm = 0;
        command.right_pwm = 0;
        return command;
    }

    int16_t error = estimate->position;
    pid->integral += error;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }

    int16_t derivative = (int16_t)(error - pid->previous_error);
    pid->previous_error = error;

    int32_t turn = ((int32_t)pid->kp * error) +
                   ((int32_t)pid->ki * pid->integral) +
                   ((int32_t)pid->kd * derivative);
    turn /= 1000;

    command.left_pwm = clamp_i16((int32_t)base_pwm + turn, -ROBOT_MAX_PWM, ROBOT_MAX_PWM);
    command.right_pwm = clamp_i16((int32_t)base_pwm - turn, -ROBOT_MAX_PWM, ROBOT_MAX_PWM);
    return command;
}
