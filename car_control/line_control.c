#include "line_control.h"

#include "gray_sensor.h"
#include "pid.h"
#include "speed_control.h"

#if (LINE_CONTROL_SWAP_MOTOR_CHANNELS > 1U)
#error "LINE_CONTROL_SWAP_MOTOR_CHANNELS must be 0 or 1"
#endif

/*
 * 八路探头从左到右的位置权重。
 */
static const int16_t g_lineWeights[8] = {
    -3500, -2500, -1500, -500,
      500,  1500,  2500, 3500
};

static PID_t g_linePID;
static float g_baseSpeed;
static LineControl_Status_t g_status;

/*
 * 最近一次黑线所在方向：
 *
 *  1：右侧；
 * -1：左侧；
 *  0：未知。
 */
static int8_t g_lastLineDirection;

/*
 * 直角转弯状态变量。
 */
static uint8_t g_cornerActive;
static int8_t g_cornerDirection;
static uint16_t g_cornerCycles;

/*
 * 是否已经离开进入弯道前的旧黑线。
 */
static uint8_t g_cornerLeftOldLine;

/*
 * 中心探头连续没有检测到线的次数。
 */
static uint8_t g_cornerLeaveStableCycles;

/*
 * 离开旧线后，中心探头连续找到新线的次数。
 */
static uint8_t g_cornerExitStableCycles;

/*
 * 退出直角弯后的保护时间。
 */
static uint16_t g_cornerCooldownCycles;

static float LineControl_Limit(
    float value,
    float minimum,
    float maximum)
{
    if (value > maximum)
    {
        return maximum;
    }

    if (value < minimum)
    {
        return minimum;
    }

    return value;
}

static uint8_t LineControl_GetLineBits(uint8_t raw_sensor)
{
#if LINE_CONTROL_TRACK_BLACK_LINE
    return (uint8_t)(~raw_sensor);
#else
    return raw_sensor;
#endif
}

static float LineControl_GetPosition(
    uint8_t line_bits,
    uint8_t *active_count)
{
    int32_t weighted_sum = 0;
    uint8_t count = 0U;
    uint8_t bit;

    for (bit = 0U; bit < 8U; bit++)
    {
        if ((line_bits & (uint8_t)(1U << bit)) != 0U)
        {
#if LINE_CONTROL_BIT0_IS_LEFT
            weighted_sum += g_lineWeights[bit];
#else
            weighted_sum += g_lineWeights[7U - bit];
#endif
            count++;
        }
    }

    *active_count = count;

    if (count == 0U)
    {
        return 0.0f;
    }

    return
        ((float)weighted_sum / (float)count) /
        3500.0f;
}

/*
 * 检测中心探头。
 *
 * bit3和bit4是中间两个探头。
 */
static uint8_t LineControl_CenterDetected(uint8_t line_bits)
{
    return ((line_bits & 0x18U) != 0U) ? 1U : 0U;
}

/*
 * 根据方向设置直角转弯速度。
 */
static void LineControl_GetCornerTargets(
    int8_t direction,
    float *left_target,
    float *right_target)
{
    if (direction > 0)
    {
        /*
         * 向右转：
         * 左轮前进，右轮反转。
         */
        *left_target  = LINE_CONTROL_CORNER_OUTER_SPEED;
        *right_target = LINE_CONTROL_CORNER_INNER_SPEED;
    }
    else
    {
        /*
         * 向左转：
         * 左轮反转，右轮前进。
         */
        *left_target  = LINE_CONTROL_CORNER_INNER_SPEED;
        *right_target = LINE_CONTROL_CORNER_OUTER_SPEED;
    }
}

static void LineControl_ApplyWheelTargets(
    float physical_left_target,
    float physical_right_target)
{
#if LINE_CONTROL_SWAP_MOTOR_CHANNELS
    SpeedControl_SetTarget(
        physical_right_target,
        physical_left_target);
#else
    SpeedControl_SetTarget(
        physical_left_target,
        physical_right_target);
#endif
}

/*
 * 开始直角转弯。
 */
static void LineControl_StartCorner(
    int8_t direction,
    float position)
{
    float left_target;
    float right_target;

    g_cornerActive            = 1U;
    g_cornerDirection         = direction;
    g_cornerCycles            = 0U;
    g_cornerLeftOldLine       = 0U;
    g_cornerLeaveStableCycles = 0U;
    g_cornerExitStableCycles  = 0U;

    g_lastLineDirection = direction;

    PID_Reset(&g_linePID);

    LineControl_GetCornerTargets(
        direction,
        &left_target,
        &right_target);

    g_status.corner_active        = 1U;
    g_status.corner_direction     = direction;
    g_status.corner_left_old_line = 0U;
    g_status.corner_count         = 0U;
    g_status.line_detected        = 1U;
    g_status.position             = position;
    g_status.correction =
        (float)direction * LINE_CONTROL_MAX_CORRECTION;
    g_status.left_target  = left_target;
    g_status.right_target = right_target;

    LineControl_ApplyWheelTargets(
        left_target,
        right_target);
}

/*
 * 更新直角转弯状态。
 *
 * 返回1：直角弯仍然接管电机；
 * 返回0：已经找到新线，恢复普通循迹。
 */
static uint8_t LineControl_UpdateCorner(
    uint8_t line_bits,
    uint8_t active_count,
    float position)
{
    float left_target;
    float right_target;
    uint8_t center_detected;

    g_cornerCycles++;

    center_detected =
        LineControl_CenterDetected(line_bits);

    g_status.corner_active    = 1U;
    g_status.corner_direction = g_cornerDirection;
    g_status.corner_count     = g_cornerCycles;
    g_status.position         = position;
    g_status.line_detected =
        (active_count > 0U) ? 1U : 0U;

    /*
     * 第一阶段：
     * 必须先确认中心探头离开了旧黑线。
     *
     * 只有经过最短转弯时间后，才开始判断。
     */
    if ((g_cornerLeftOldLine == 0U) &&
        (g_cornerCycles >= LINE_CONTROL_CORNER_MIN_CYCLES))
    {
        if (center_detected == 0U)
        {
            g_cornerLeaveStableCycles++;

            if (g_cornerLeaveStableCycles >=
                LINE_CONTROL_CORNER_LEAVE_STABLE_CYCLES)
            {
                g_cornerLeftOldLine       = 1U;
                g_cornerExitStableCycles  = 0U;
            }
        }
        else
        {
            g_cornerLeaveStableCycles = 0U;
        }
    }

    /*
     * 第二阶段：
     * 只有确认离开旧线后，才允许寻找转弯后的新线。
     */
    if (g_cornerLeftOldLine != 0U)
    {
        /*
         * 中心探头找到黑线，并且不是整排都压在宽黑线上。
         */
        if ((center_detected != 0U) &&
            (active_count > 0U) &&
            (active_count <= 4U))
        {
            g_cornerExitStableCycles++;
        }
        else
        {
            g_cornerExitStableCycles = 0U;
        }
    }

    g_status.corner_left_old_line =
        g_cornerLeftOldLine;

    /*
     * 连续稳定找到新线，转弯完成。
     */
    if (g_cornerExitStableCycles >=
        LINE_CONTROL_CORNER_EXIT_STABLE_CYCLES)
    {
        g_cornerActive            = 0U;
        g_cornerDirection         = 0;
        g_cornerCycles            = 0U;
        g_cornerLeftOldLine       = 0U;
        g_cornerLeaveStableCycles = 0U;
        g_cornerExitStableCycles  = 0U;

        g_cornerCooldownCycles =
            LINE_CONTROL_CORNER_COOLDOWN_CYCLES;

        g_status.corner_active        = 0U;
        g_status.corner_direction     = 0;
        g_status.corner_left_old_line = 0U;
        g_status.corner_count         = 0U;
        g_status.corner_cooldown =
            g_cornerCooldownCycles;
        g_status.lost_count = 0U;

        PID_Reset(&g_linePID);

        return 0U;
    }

    /*
     * 转弯超时仍找不到新线，停车。
     */
    if (g_cornerCycles >=
        LINE_CONTROL_CORNER_MAX_CYCLES)
    {
        g_cornerActive            = 0U;
        g_cornerDirection         = 0;
        g_cornerCycles            = 0U;
        g_cornerLeftOldLine       = 0U;
        g_cornerLeaveStableCycles = 0U;
        g_cornerExitStableCycles  = 0U;

        g_status.corner_active        = 0U;
        g_status.corner_direction     = 0;
        g_status.corner_left_old_line = 0U;
        g_status.corner_count         = 0U;
        g_status.correction           = 0.0f;
        g_status.left_target          = 0.0f;
        g_status.right_target         = 0.0f;

        SpeedControl_Stop();

        return 1U;
    }

    /*
     * 转弯未完成时，始终保持最初的转弯方向。
     * 这里绝对不根据position改变方向。
     */
    LineControl_GetCornerTargets(
        g_cornerDirection,
        &left_target,
        &right_target);

    g_status.correction =
        (float)g_cornerDirection *
        LINE_CONTROL_MAX_CORRECTION;
    g_status.left_target  = left_target;
    g_status.right_target = right_target;

    LineControl_ApplyWheelTargets(
        left_target,
        right_target);

    return 1U;
}

void LineControl_Init(void)
{
    PID_Init(
        &g_linePID,
        LINE_CONTROL_KP,
        LINE_CONTROL_KI,
        LINE_CONTROL_KD,
        LINE_CONTROL_PERIOD_SECONDS,
        -LINE_CONTROL_MAX_CORRECTION,
        LINE_CONTROL_MAX_CORRECTION);

    g_baseSpeed = LINE_CONTROL_DEFAULT_BASE_SPEED;

    g_lastLineDirection = 0;

    g_cornerActive            = 0U;
    g_cornerDirection         = 0;
    g_cornerCycles            = 0U;
    g_cornerLeftOldLine       = 0U;
    g_cornerLeaveStableCycles = 0U;
    g_cornerExitStableCycles  = 0U;
    g_cornerCooldownCycles    = 0U;

    g_status.raw_sensor          = 0xFFU;
    g_status.line_bits           = 0U;
    g_status.active_count        = 0U;
    g_status.line_detected       = 0U;
    g_status.corner_active       = 0U;
    g_status.corner_direction    = 0;
    g_status.corner_left_old_line = 0U;
    g_status.update_count        = 0U;
    g_status.lost_count          = 0U;
    g_status.corner_count        = 0U;
    g_status.corner_cooldown     = 0U;
    g_status.position            = 0.0f;
    g_status.correction          = 0.0f;
    g_status.left_target         = 0.0f;
    g_status.right_target        = 0.0f;

    SpeedControl_Stop();
}

void LineControl_Update(void)
{
    uint8_t raw_sensor;
    uint8_t line_bits;
    uint8_t active_count;

    float position;
    float correction;
    float left_target;
    float right_target;

    raw_sensor = GraySensor_Read();
    line_bits = LineControl_GetLineBits(raw_sensor);

    position = LineControl_GetPosition(
        line_bits,
        &active_count);

    g_status.raw_sensor   = raw_sensor;
    g_status.line_bits    = line_bits;
    g_status.active_count = active_count;
    g_status.update_count++;

    /*
     * 更新直角弯退出后的保护倒计时。
     */
    if (g_cornerCooldownCycles > 0U)
    {
        g_cornerCooldownCycles--;
    }

    g_status.corner_cooldown =
        g_cornerCooldownCycles;

    /*
     * 已经进入直角弯后，由直角弯状态完全接管。
     */
    if (g_cornerActive != 0U)
    {
        if (LineControl_UpdateCorner(
                line_bits,
                active_count,
                position) != 0U)
        {
            return;
        }
    }

    /*
     * 没有探头检测到黑线。
     */
    if (active_count == 0U)
    {
        g_status.line_detected = 0U;
        g_status.lost_count++;
        g_status.position   = 0.0f;
        g_status.correction = 0.0f;

        PID_Reset(&g_linePID);

        /*
         * 保护期内丢线时，只按照最近方向低速搜索，
         * 不允许触发反方向直角弯。
         */
        if ((g_lastLineDirection != 0) &&
            (g_status.lost_count <=
             LINE_CONTROL_SEARCH_MAX_CYCLES))
        {
            LineControl_GetCornerTargets(
                g_lastLineDirection,
                &left_target,
                &right_target);

            g_status.correction =
                (float)g_lastLineDirection *
                LINE_CONTROL_MAX_CORRECTION;
            g_status.left_target  = left_target;
            g_status.right_target = right_target;

            LineControl_ApplyWheelTargets(
                left_target,
                right_target);
        }
        else
        {
            g_status.left_target  = 0.0f;
            g_status.right_target = 0.0f;

            SpeedControl_Stop();
        }

        return;
    }

    g_status.line_detected = 1U;
    g_status.lost_count = 0U;

    /*
     * 只有不在保护期时，才更新最近黑线方向。
     * 防止刚出弯时短暂偏差覆盖原来的转弯方向。
     */
    if (g_cornerCooldownCycles == 0U)
    {
        if (position > LINE_CONTROL_DIRECTION_THRESHOLD)
        {
            g_lastLineDirection = 1;
        }
        else if (position < -LINE_CONTROL_DIRECTION_THRESHOLD)
        {
            g_lastLineDirection = -1;
        }
    }

    /*
     * 只有保护倒计时结束后，才能进入下一个直角弯。
     */
    if (g_cornerCooldownCycles == 0U)
    {
        if (position >=
            LINE_CONTROL_CORNER_POSITION_THRESHOLD)
        {
            LineControl_StartCorner(1, position);
            return;
        }

        if (position <=
            -LINE_CONTROL_CORNER_POSITION_THRESHOLD)
        {
            LineControl_StartCorner(-1, position);
            return;
        }
    }

    /*
     * 普通PID循迹。
     */
    correction = PID_Calculate(
        &g_linePID,
        position,
        0.0f);

    left_target  = g_baseSpeed + correction;
    right_target = g_baseSpeed - correction;

    /*
     * 普通循迹期间不允许反转。
     */
    left_target = LineControl_Limit(
        left_target,
        0.0f,
        LINE_CONTROL_MAX_WHEEL_SPEED);

    right_target = LineControl_Limit(
        right_target,
        0.0f,
        LINE_CONTROL_MAX_WHEEL_SPEED);

    g_status.corner_active        = 0U;
    g_status.corner_direction     = 0;
    g_status.corner_left_old_line = 0U;
    g_status.corner_count         = 0U;
    g_status.position             = position;
    g_status.correction           = correction;
    g_status.left_target          = left_target;
    g_status.right_target         = right_target;

    LineControl_ApplyWheelTargets(
        left_target,
        right_target);
}

void LineControl_Stop(void)
{
    PID_Reset(&g_linePID);

    g_lastLineDirection = 0;

    g_cornerActive            = 0U;
    g_cornerDirection         = 0;
    g_cornerCycles            = 0U;
    g_cornerLeftOldLine       = 0U;
    g_cornerLeaveStableCycles = 0U;
    g_cornerExitStableCycles  = 0U;
    g_cornerCooldownCycles    = 0U;

    g_status.line_detected        = 0U;
    g_status.corner_active        = 0U;
    g_status.corner_direction     = 0;
    g_status.corner_left_old_line = 0U;
    g_status.lost_count           = 0U;
    g_status.corner_count         = 0U;
    g_status.corner_cooldown      = 0U;
    g_status.correction           = 0.0f;
    g_status.left_target          = 0.0f;
    g_status.right_target         = 0.0f;

    SpeedControl_Stop();
}

void LineControl_SetBaseSpeed(float base_speed)
{
    g_baseSpeed = LineControl_Limit(
        base_speed,
        0.0f,
        LINE_CONTROL_MAX_WHEEL_SPEED);
}

void LineControl_SetTunings(
    float kp,
    float ki,
    float kd)
{
    PID_Init(
        &g_linePID,
        kp,
        ki,
        kd,
        LINE_CONTROL_PERIOD_SECONDS,
        -LINE_CONTROL_MAX_CORRECTION,
        LINE_CONTROL_MAX_CORRECTION);
}

void LineControl_GetStatus(
    LineControl_Status_t *status)
{
    if (status == 0)
    {
        return;
    }

    *status = g_status;
}