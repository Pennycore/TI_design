#include "robot_app.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "board_config.h"
#include "hal.h"
#include "motor_control.h"

#define BALL_FLAG_DETECTED 0x01U
#define BALL_FLAG_STABLE 0x02U
#define BALL_FLAG_CLOSE 0x04U

static int16_t clamp_drive(int32_t value)
{
    if (value > ROBOT_MAX_PWM) {
        return ROBOT_MAX_PWM;
    }
    if (value < -ROBOT_MAX_PWM) {
        return -ROBOT_MAX_PWM;
    }
    return (int16_t)value;
}

static bool ball_is_recent(const robot_app_t *app, uint32_t now_ms)
{
    return (now_ms - app->last_ball_ms) <= ROBOT_BALL_LOST_TIMEOUT_MS;
}

static bool ball_is_detected(const robot_app_t *app, uint32_t now_ms)
{
    return ball_is_recent(app, now_ms) &&
           ((app->last_ball.flags & BALL_FLAG_DETECTED) != 0U) &&
           (app->last_ball.confidence >= ROBOT_BALL_MIN_CONFIDENCE);
}

static bool ball_is_close(const robot_app_t *app)
{
    return ((app->last_ball.flags & BALL_FLAG_CLOSE) != 0U) ||
           (app->last_ball.radius >= ROBOT_BALL_STOP_RADIUS);
}

static drive_command_t visual_align_command(const vision_ball_t *ball)
{
    int32_t turn = (int32_t)ball->offset_x * ROBOT_VISUAL_KP;
    drive_command_t command;

    command.left_pwm = clamp_drive((int32_t)ROBOT_ALIGN_PWM + turn);
    command.right_pwm = clamp_drive((int32_t)ROBOT_ALIGN_PWM - turn);
    return command;
}

static void send_telemetry(robot_app_t *app, const gw_gray_sample_t *sample, uint32_t now_ms)
{
    uint8_t payload[12];
    uint8_t frame[24];
    mcu_telemetry_t telemetry;

    if ((now_ms - app->last_telemetry_ms) < ROBOT_TELEMETRY_PERIOD_MS) {
        return;
    }

    telemetry.time_ms = now_ms;
    telemetry.line_pos = app->last_line.position;
    telemetry.left_pwm = app->last_drive.left_pwm;
    telemetry.right_pwm = app->last_drive.right_pwm;
    telemetry.state = (uint8_t)app->state;
    telemetry.gray_bits = sample->digital_valid ? sample->digital_bits : app->gray_bits;

    uint8_t len = serial_encode_mcu_telemetry_payload(&telemetry, payload);
    size_t frame_len = serial_encode(MSG_MCU_TELEMETRY,
                                     app->telemetry_seq++,
                                     payload,
                                     len,
                                     frame,
                                     sizeof(frame));
    if (frame_len > 0U) {
        (void)hal_uart_write(HAL_UART_K230, frame, frame_len);
    }

    app->last_telemetry_ms = now_ms;
}

void robot_app_init(robot_app_t *app)
{
    memset(app, 0, sizeof(*app));

    hal_init();
    serial_parser_init(&app->serial_parser);
    line_pid_init(&app->line_pid);
    motor_init();

    app->state = ROBOT_STATE_BOOT;
    if (gw_gray_init(&app->gray)) {
        app->state = ROBOT_STATE_LINE_FOLLOW;
        hal_log("GW gray sensor ready\r\n");
    } else {
        app->state = ROBOT_STATE_SENSOR_FAULT;
        motor_stop();
        hal_log("GW gray sensor not found\r\n");
    }
}

void robot_app_poll_uart(robot_app_t *app)
{
    uint8_t bytes[32];
    serial_frame_t frame;
    size_t count = hal_uart_read(HAL_UART_K230, bytes, sizeof(bytes));

    for (size_t i = 0U; i < count; ++i) {
        if (!serial_parser_push(&app->serial_parser, bytes[i], &frame)) {
            continue;
        }

        if (frame.msg_id == MSG_VISION_BALL) {
            vision_ball_t ball;
            if (serial_decode_vision_ball(&frame, &ball)) {
                app->last_ball = ball;
                app->last_ball_ms = hal_millis();
            }
        }
    }
}

void robot_app_tick(robot_app_t *app)
{
    gw_gray_sample_t sample;
    uint32_t now_ms = hal_millis();
    line_estimate_t line;
    drive_command_t drive = {0};

    if ((now_ms - app->last_tick_ms) < ROBOT_CONTROL_PERIOD_MS) {
        return;
    }
    app->last_tick_ms = now_ms;

    if (app->state == ROBOT_STATE_SENSOR_FAULT) {
        motor_stop();
        return;
    }

    if (!gw_gray_read_sample(&app->gray, &sample)) {
        app->state = ROBOT_STATE_SENSOR_FAULT;
        motor_stop();
        hal_log("GW gray sample failed\r\n");
        return;
    }

    if (sample.digital_valid) {
        app->gray_bits = sample.digital_bits;
    }

    if (sample.analog_valid) {
        line = line_estimate_from_analog(sample.analog, ROBOT_BLACK_LINE != 0);
    } else {
        line = line_estimate_from_digital(sample.digital_bits, ROBOT_BLACK_LINE != 0);
    }

    app->last_line = line;

    if (!line.valid) {
        app->state = ROBOT_STATE_STOPPED;
        drive.left_pwm = 0;
        drive.right_pwm = 0;
    } else if (ball_is_detected(app, now_ms)) {
        if (ball_is_close(app)) {
            app->state = ROBOT_STATE_STOPPED;
            drive.left_pwm = 0;
            drive.right_pwm = 0;
        } else {
            app->state = ROBOT_STATE_BALL_ALIGN;
            drive = visual_align_command(&app->last_ball);
        }
    } else {
        app->state = ROBOT_STATE_LINE_FOLLOW;
        drive = line_pid_update(&app->line_pid, &line, ROBOT_BASE_PWM);
    }

    app->last_drive = drive;
    motor_apply(drive.left_pwm, drive.right_pwm);
    send_telemetry(app, &sample, now_ms);
}
