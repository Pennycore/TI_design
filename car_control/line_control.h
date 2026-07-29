#ifndef LINE_CONTROL_H_
#define LINE_CONTROL_H_

#include <stdint.h>

/* The gray sensor is sampled about every 10 ms. */
#define LINE_CONTROL_PERIOD_SECONDS          (0.010f)

/*
 * Wheel speed unit: quadrature encoder counts per 10 ms.
 *
 * These conservative defaults are intended to make ordinary tracking work
 * before increasing speed.  The R35 cm ends of the competition track do not
 * need any right-angle or pivot-turn state machine.
 */
#define LINE_CONTROL_DEFAULT_BASE_SPEED      (3.0f)
#define LINE_CONTROL_MIN_BASE_SPEED          (2.2f)
#define LINE_CONTROL_MAX_WHEEL_SPEED         (6.0f)
#define LINE_CONTROL_MAX_CORRECTION          (3.0f)
#define LINE_CONTROL_CURVE_SLOWDOWN          (0.8f)

/*
 * Position PID.  Position is normalized to -1.0 (left) ... +1.0 (right).
 * Integral is intentionally disabled for line tracking.
 */
#define LINE_CONTROL_KP                      (3.2f)
#define LINE_CONTROL_KI                      (0.0f)
#define LINE_CONTROL_KD                      (0.015f)

/*
 * Low-pass filter coefficient for the newest position sample.
 * 1.0 disables filtering; a smaller value filters more strongly.
 */
#define LINE_CONTROL_POSITION_FILTER_ALPHA   (0.70f)
#define LINE_CONTROL_DIRECTION_THRESHOLD     (0.08f)

/*
 * A one- or two-frame sensor dropout should not stop the car.  If the line is
 * still absent, steer forward in the last known direction for at most 250 ms,
 * then stop.  The inner wheel never reverses during ordinary tracking.
 */
#define LINE_CONTROL_LOST_HOLD_CYCLES        (2U)
#define LINE_CONTROL_SEARCH_MAX_CYCLES       (25U)
#define LINE_CONTROL_SEARCH_OUTER_SPEED      (2.5f)
#define LINE_CONTROL_SEARCH_INNER_SPEED      (0.6f)

/*
 * Sensor installation:
 * - The on-car steering test showed that bit 0 is on the physical left:
 *   a line on the left must slow the left wheel and speed the right wheel.
 * - The competition track is a black line on a white background.  The current
 *   sensor protocol reports white as 1 and black as 0, so TRACK_BLACK_LINE=1
 *   inverts the raw byte before calculating position.
 */
#define LINE_CONTROL_BIT0_IS_LEFT            (1U)
#define LINE_CONTROL_TRACK_BLACK_LINE        (1U)

/*
 * Bench test result:
 * driver/encoder channel A is the physical left wheel and channel B is the
 * physical right wheel, so physical wheel requests must not be swapped.
 */
#define LINE_CONTROL_SWAP_MOTOR_CHANNELS     (0U)

typedef struct
{
    uint8_t raw_sensor;
    uint8_t line_bits;
    uint8_t active_count;
    uint8_t line_detected;
    uint8_t search_active;
    int8_t last_direction;
    uint32_t update_count;
    uint32_t lost_count;
    float raw_position;
    float position;
    float correction;
    float left_target;
    float right_target;
} LineControl_Status_t;

void LineControl_Init(void);
void LineControl_Update(void);
void LineControl_Stop(void);
void LineControl_SetBaseSpeed(float base_speed);
void LineControl_SetTunings(float kp, float ki, float kd);
void LineControl_GetStatus(LineControl_Status_t *status);

#endif /* LINE_CONTROL_H_ */
