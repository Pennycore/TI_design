#ifndef LINE_CONTROL_H_
#define LINE_CONTROL_H_

#include <stdint.h>

<<<<<<< HEAD
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
=======
/*
 * 循迹控制周期约为10ms。
 */
#define LINE_CONTROL_PERIOD_SECONDS (0.010f)

/*
 * 普通直线速度。
 * 车身长32cm、宽24cm，先使用较低速度测试。
 */
#define LINE_CONTROL_DEFAULT_BASE_SPEED (2.5f)
#define LINE_CONTROL_MAX_WHEEL_SPEED    (6.0f)
#define LINE_CONTROL_MAX_CORRECTION     (3.0f)

/*
 * 黑线位置达到左右外侧时，进入直角弯。
 */
#define LINE_CONTROL_CORNER_POSITION_THRESHOLD (0.60f)
#define LINE_CONTROL_DIRECTION_THRESHOLD       (0.05f)

/*
 * 直角转弯速度：
 *
 * 右转：左轮+4，右轮-3；
 * 左转：左轮-3，右轮+4。
 */
#define LINE_CONTROL_CORNER_OUTER_SPEED (4.0f)
#define LINE_CONTROL_CORNER_INNER_SPEED (-3.0f)

/*
 * 直角弯状态参数。
 *
 * 控制周期约为10ms：
 * 20次约为200ms；
 * 150次约为1.5秒；
 * 25次约为250ms。
 */
#define LINE_CONTROL_CORNER_MIN_CYCLES          (20U)
#define LINE_CONTROL_CORNER_MAX_CYCLES          (150U)

/*
 * 中间探头连续离开旧线3次，
 * 才认为车头已经离开旧方向。
 */
#define LINE_CONTROL_CORNER_LEAVE_STABLE_CYCLES (3U)

/*
 * 离开旧线后，中间探头连续检测到新线4次，
 * 才认为直角转弯完成。
 */
#define LINE_CONTROL_CORNER_EXIT_STABLE_CYCLES  (4U)

/*
 * 退出直角弯后的保护时间。
 * 保护期内禁止再次进入直角弯，防止反向误触发。
 */
#define LINE_CONTROL_CORNER_COOLDOWN_CYCLES     (25U)

/*
 * 普通循迹丢线后的搜索时间。
 */
#define LINE_CONTROL_SEARCH_MAX_CYCLES (100U)

/*
 * 普通循迹PID参数。
 */
#define LINE_CONTROL_KP (4.0f)
#define LINE_CONTROL_KI (0.0f)
#define LINE_CONTROL_KD (0.025f)

/*
 * bit0是否对应车身最左侧探头。
 */
#define LINE_CONTROL_BIT0_IS_LEFT (1U)

/*
 * 白底黑线模式。
 */
#define LINE_CONTROL_TRACK_BLACK_LINE (1U)

/*
 * 已经通过实车转向测试确认，左右电机通道不交换。
 */
#define LINE_CONTROL_SWAP_MOTOR_CHANNELS (0U)
>>>>>>> c4c408127b779efd2b8ce6b4ef4b3f90f99e0207

typedef struct
{
    uint8_t raw_sensor;
    uint8_t line_bits;
    uint8_t active_count;
    uint8_t line_detected;
<<<<<<< HEAD
    uint8_t search_active;
    int8_t last_direction;
    uint32_t update_count;
    uint32_t lost_count;
    float raw_position;
=======

    uint8_t corner_active;
    int8_t corner_direction;
    uint8_t corner_left_old_line;

    uint32_t update_count;
    uint32_t lost_count;
    uint32_t corner_count;
    uint32_t corner_cooldown;

>>>>>>> c4c408127b779efd2b8ce6b4ef4b3f90f99e0207
    float position;
    float correction;
    float left_target;
    float right_target;
} LineControl_Status_t;

void LineControl_Init(void);
<<<<<<< HEAD
void LineControl_Update(void);
void LineControl_Stop(void);
void LineControl_SetBaseSpeed(float base_speed);
void LineControl_SetTunings(float kp, float ki, float kd);
=======

void LineControl_Update(void);

void LineControl_Stop(void);

void LineControl_SetBaseSpeed(float base_speed);

void LineControl_SetTunings(float kp, float ki, float kd);

>>>>>>> c4c408127b779efd2b8ce6b4ef4b3f90f99e0207
void LineControl_GetStatus(LineControl_Status_t *status);

#endif /* LINE_CONTROL_H_ */