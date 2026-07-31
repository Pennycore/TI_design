#include "system_control.h"

/*
 * 控制周期推荐为10ms。
 *
 * 离开A点后，在这段时间内不接受新的A点触发，
 * 防止启动时A点标记一直位于灰度传感器下方，
 * 被立即误判为已经完成一圈。
 */
#define SYSTEM_LEAVE_A_MIN_TIME_MS       (1000U)

/*
 * 如果已经完全离开A点标记，则不必等满上面的时间。
 * 连续检测不到A点达到该时间，即认为已离开A点。
 */
#define SYSTEM_A_CLEAR_CONFIRM_MS        (200U)

/*
 * B点检测必须连续有效一段时间，防止噪声误触发。
 */
#define SYSTEM_B_CONFIRM_TIME_MS         (30U)

/*
 * 返回A点时，同样要求连续检测到A点一段时间。
 */
#define SYSTEM_A_CONFIRM_TIME_MS         (30U)

/*
 * 检测到终点后保留一段制动时间。
 * 后续可以根据实际停车距离调整。
 */
#define SYSTEM_BRAKE_TIME_MS             (500U)

/*
 * 连续丢线超过该时间才进入故障。
 * 短时间丢线由循迹模块自己寻找轨迹。
 */
#define SYSTEM_LINE_LOST_ERROR_MS        (1500U)

/*
 * 一次任务的最大运行时间。
 * 题目高分项要求30秒内完成，因此这里先留到35秒，
 * 防止程序因为轻微超时立即失控。
 */
#define SYSTEM_RUN_TIMEOUT_MS            (35000U)

typedef struct
{
    SystemState state;
    SystemError error;

    uint32_t state_time_ms;
    uint32_t elapsed_time_ms;
    uint32_t b_time_ms;
    uint32_t finish_time_ms;

    uint32_t a_clear_time_ms;
    uint32_t a_confirm_time_ms;
    uint32_t b_confirm_time_ms;
    uint32_t line_lost_time_ms;

    uint8_t previous_marker_a;
    uint8_t previous_marker_b;

    uint8_t a_has_been_cleared;
    uint8_t b_has_been_passed;
} SystemControlData;

static SystemControlData g_system;

/*
 * 安全的无符号计时累加。
 */
static uint32_t SystemControl_AddTime(uint32_t value,
                                      uint32_t delta_ms)
{
    if (value > (0xFFFFFFFFU - delta_ms))
    {
        return 0xFFFFFFFFU;
    }

    return value + delta_ms;
}

/*
 * 切换状态，并清零当前状态计时。
 */
static void SystemControl_ChangeState(SystemState new_state)
{
    g_system.state = new_state;
    g_system.state_time_ms = 0U;
}

/*
 * 进入故障状态。
 */
static void SystemControl_EnterError(SystemError error)
{
    g_system.error = error;
    SystemControl_ChangeState(SYSTEM_STATE_ERROR);
}

void SystemControl_Init(void)
{
    SystemControl_Reset();
}

void SystemControl_Reset(void)
{
    g_system.state = SYSTEM_STATE_WAIT_START;
    g_system.error = SYSTEM_ERROR_NONE;

    g_system.state_time_ms = 0U;
    g_system.elapsed_time_ms = 0U;
    g_system.b_time_ms = 0U;
    g_system.finish_time_ms = 0U;

    g_system.a_clear_time_ms = 0U;
    g_system.a_confirm_time_ms = 0U;
    g_system.b_confirm_time_ms = 0U;
    g_system.line_lost_time_ms = 0U;

    g_system.previous_marker_a = 0U;
    g_system.previous_marker_b = 0U;

    g_system.a_has_been_cleared = 0U;
    g_system.b_has_been_passed = 0U;
}

void SystemControl_Update(const SystemControlInput *input,
                          uint32_t delta_ms)
{
    uint8_t marker_a_rising;
    uint8_t marker_b_rising;

    if ((input == 0) || (delta_ms == 0U))
    {
        return;
    }

    /*
     * 记录标记信号的上升沿。
     *
     * 当前版本主要使用连续确认时间判断，
     * 上升沿保留下来便于后续调试和扩展。
     */
    marker_a_rising =
        (uint8_t)((input->marker_a != 0U) &&
                  (g_system.previous_marker_a == 0U));

    marker_b_rising =
        (uint8_t)((input->marker_b != 0U) &&
                  (g_system.previous_marker_b == 0U));

    g_system.previous_marker_a =
        (input->marker_a != 0U) ? 1U : 0U;

    g_system.previous_marker_b =
        (input->marker_b != 0U) ? 1U : 0U;

    /*
     * 避免编译器对暂未使用变量报警。
     * 后续可以用这两个上升沿输出调试信息。
     */
    (void)marker_a_rising;
    (void)marker_b_rising;

    g_system.state_time_ms =
        SystemControl_AddTime(g_system.state_time_ms, delta_ms);

    /*
     * 等待、完成、故障状态不累计比赛运行时间。
     */
    if ((g_system.state != SYSTEM_STATE_WAIT_START) &&
        (g_system.state != SYSTEM_STATE_FINISHED) &&
        (g_system.state != SYSTEM_STATE_ERROR))
    {
        g_system.elapsed_time_ms =
            SystemControl_AddTime(g_system.elapsed_time_ms, delta_ms);
    }

    /*
     * 运行状态下检查传感器故障、任务超时和长时间丢线。
     */
    if ((g_system.state != SYSTEM_STATE_WAIT_START) &&
        (g_system.state != SYSTEM_STATE_FINISHED) &&
        (g_system.state != SYSTEM_STATE_ERROR))
    {
        if (input->sensor_error != 0U)
        {
            SystemControl_EnterError(SYSTEM_ERROR_SENSOR);
            return;
        }

        if (g_system.elapsed_time_ms >= SYSTEM_RUN_TIMEOUT_MS)
        {
            SystemControl_EnterError(SYSTEM_ERROR_TIMEOUT);
            return;
        }

        if (input->line_lost != 0U)
        {
            g_system.line_lost_time_ms =
                SystemControl_AddTime(
                    g_system.line_lost_time_ms,
                    delta_ms);

            if (g_system.line_lost_time_ms >=
                SYSTEM_LINE_LOST_ERROR_MS)
            {
                SystemControl_EnterError(
                    SYSTEM_ERROR_LINE_LOST);
                return;
            }
        }
        else
        {
            g_system.line_lost_time_ms = 0U;
        }
    }
    else
    {
        g_system.line_lost_time_ms = 0U;
    }

    switch (g_system.state)
    {
        case SYSTEM_STATE_WAIT_START:
        {
            /*
             * 等待启动。
             *
             * 后续按键到货后，start_pressed接按键按下事件。
             * 现在也可以在main中用上电3秒后产生一次启动事件。
             */
            if (input->start_pressed != 0U)
            {
                g_system.error = SYSTEM_ERROR_NONE;

                g_system.elapsed_time_ms = 0U;
                g_system.b_time_ms = 0U;
                g_system.finish_time_ms = 0U;

                g_system.a_clear_time_ms = 0U;
                g_system.a_confirm_time_ms = 0U;
                g_system.b_confirm_time_ms = 0U;
                g_system.line_lost_time_ms = 0U;

                g_system.a_has_been_cleared = 0U;
                g_system.b_has_been_passed = 0U;

                SystemControl_ChangeState(
                    SYSTEM_STATE_LEAVE_A);
            }
            break;
        }

        case SYSTEM_STATE_LEAVE_A:
        {
            /*
             * 检测是否已经离开A点区域。
             */
            if (input->marker_a == 0U)
            {
                g_system.a_clear_time_ms =
                    SystemControl_AddTime(
                        g_system.a_clear_time_ms,
                        delta_ms);

                if (g_system.a_clear_time_ms >=
                    SYSTEM_A_CLEAR_CONFIRM_MS)
                {
                    g_system.a_has_been_cleared = 1U;
                }
            }
            else
            {
                g_system.a_clear_time_ms = 0U;
            }

            /*
             * 满足以下任一条件即进入前往B点状态：
             *
             * 1. A点已经连续消失，确认车辆离开；
             * 2. 启动后已经过去最小保护时间。
             */
            if ((g_system.a_has_been_cleared != 0U) ||
                (g_system.state_time_ms >=
                 SYSTEM_LEAVE_A_MIN_TIME_MS))
            {
                SystemControl_ChangeState(
                    SYSTEM_STATE_RUN_TO_B);
            }
            break;
        }

        case SYSTEM_STATE_RUN_TO_B:
        {
            /*
             * 连续检测B点，避免单次噪声触发。
             */
            if (input->marker_b != 0U)
            {
                g_system.b_confirm_time_ms =
                    SystemControl_AddTime(
                        g_system.b_confirm_time_ms,
                        delta_ms);

                if (g_system.b_confirm_time_ms >=
                    SYSTEM_B_CONFIRM_TIME_MS)
                {
                    g_system.b_has_been_passed = 1U;

                    if (g_system.b_time_ms == 0U)
                    {
                        g_system.b_time_ms =
                            g_system.elapsed_time_ms;
                    }

                    SystemControl_ChangeState(
                        SYSTEM_STATE_PASS_B);
                }
            }
            else
            {
                g_system.b_confirm_time_ms = 0U;
            }
            break;
        }

        case SYSTEM_STATE_PASS_B:
        {
            /*
             * 等车辆完全离开B点标记后，
             * 才允许检测返回A点。
             */
            if (input->marker_b == 0U)
            {
                g_system.b_confirm_time_ms =
                    SystemControl_AddTime(
                        g_system.b_confirm_time_ms,
                        delta_ms);

                if (g_system.b_confirm_time_ms >=
                    SYSTEM_B_CONFIRM_TIME_MS)
                {
                    g_system.a_confirm_time_ms = 0U;

                    SystemControl_ChangeState(
                        SYSTEM_STATE_RETURN_TO_A);
                }
            }
            else
            {
                g_system.b_confirm_time_ms = 0U;
            }
            break;
        }

        case SYSTEM_STATE_RETURN_TO_A:
        {
            /*
             * 只有确实经过B点后，才允许把A点识别为终点。
             */
            if ((g_system.b_has_been_passed != 0U) &&
                (input->marker_a != 0U))
            {
                g_system.a_confirm_time_ms =
                    SystemControl_AddTime(
                        g_system.a_confirm_time_ms,
                        delta_ms);

                if (g_system.a_confirm_time_ms >=
                    SYSTEM_A_CONFIRM_TIME_MS)
                {
                    /*
                     * 记录第一次识别终点的时间。
                     * 制动过程不计入这里，后续可根据题目
                     * 对“停止时间”的定义再调整。
                     */
                    g_system.finish_time_ms =
                        g_system.elapsed_time_ms;

                    SystemControl_ChangeState(
                        SYSTEM_STATE_BRAKING);
                }
            }
            else
            {
                g_system.a_confirm_time_ms = 0U;
            }
            break;
        }

        case SYSTEM_STATE_BRAKING:
        {
            /*
             * 该状态下外部应停止循迹并执行制动。
             */
            if (g_system.state_time_ms >=
                SYSTEM_BRAKE_TIME_MS)
            {
                SystemControl_ChangeState(
                    SYSTEM_STATE_FINISHED);
            }
            break;
        }

        case SYSTEM_STATE_FINISHED:
        {
            /*
             * 完成后保持停车。
             * 再按一次启动键可以复位，方便重复测试。
             */
            if (input->start_pressed != 0U)
            {
                SystemControl_Reset();
            }
            break;
        }

        case SYSTEM_STATE_ERROR:
        {
            /*
             * 故障后保持停车。
             * 按键复位，但不会立即重新启动。
             */
            if (input->start_pressed != 0U)
            {
                SystemControl_Reset();
            }
            break;
        }

        default:
        {
            SystemControl_EnterError(
                SYSTEM_ERROR_SENSOR);
            break;
        }
    }
}

SystemState SystemControl_GetState(void)
{
    return g_system.state;
}

SystemError SystemControl_GetError(void)
{
    return g_system.error;
}

uint32_t SystemControl_GetElapsedTimeMs(void)
{
    return g_system.elapsed_time_ms;
}

uint32_t SystemControl_GetBTimeMs(void)
{
    return g_system.b_time_ms;
}

uint32_t SystemControl_GetFinishTimeMs(void)
{
    return g_system.finish_time_ms;
}

uint8_t SystemControl_ShouldRun(void)
{
    switch (g_system.state)
    {
        case SYSTEM_STATE_LEAVE_A:
        case SYSTEM_STATE_RUN_TO_B:
        case SYSTEM_STATE_PASS_B:
        case SYSTEM_STATE_RETURN_TO_A:
            return 1U;

        default:
            return 0U;
    }
}

uint8_t SystemControl_ShouldBrake(void)
{
    if ((g_system.state == SYSTEM_STATE_BRAKING) ||
        (g_system.state == SYSTEM_STATE_FINISHED) ||
        (g_system.state == SYSTEM_STATE_ERROR))
    {
        return 1U;
    }

    return 0U;
}

uint8_t SystemControl_IsFinished(void)
{
    return (g_system.state == SYSTEM_STATE_FINISHED)
               ? 1U
               : 0U;
}

uint8_t SystemControl_HasError(void)
{
    return (g_system.state == SYSTEM_STATE_ERROR)
               ? 1U
               : 0U;
}

const char *SystemControl_GetStateText(void)
{
    switch (g_system.state)
    {
        case SYSTEM_STATE_WAIT_START:
            return "READY";

        case SYSTEM_STATE_LEAVE_A:
            return "LEAVE A";

        case SYSTEM_STATE_RUN_TO_B:
            return "TO B";

        case SYSTEM_STATE_PASS_B:
            return "PASS B";

        case SYSTEM_STATE_RETURN_TO_A:
            return "TO A";

        case SYSTEM_STATE_BRAKING:
            return "BRAKE";

        case SYSTEM_STATE_FINISHED:
            return "FINISH";

        case SYSTEM_STATE_ERROR:
            return "ERROR";

        default:
            return "UNKNOWN";
    }
}