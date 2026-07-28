#ifndef LINE_CONTROL_H_
#define LINE_CONTROL_H_

#include <stdint.h>

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

typedef struct
{
    uint8_t raw_sensor;
    uint8_t line_bits;
    uint8_t active_count;
    uint8_t line_detected;

    uint8_t corner_active;
    int8_t corner_direction;
    uint8_t corner_left_old_line;

    uint32_t update_count;
    uint32_t lost_count;
    uint32_t corner_count;
    uint32_t corner_cooldown;

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