#include "ti_msp_dl_config.h"

#include "competition_config.h"
#include "encoder.h"
#include "gray_sensor.h"
#include "k230_uart.h"
#include "line_control.h"
#include "motor.h"
#include "oled.h"
#include "speed_control.h"
#include "stepper_encoder.h"
#include "stepper_motor.h"
#include "track_position.h"

/*
 * Temporary bench-test mode.
 *
 * IMPORTANT: lift both wheels off the ground before flashing this build.
 * Set to 0 after motor/encoder direction testing is complete.
 */
#define MOTOR_ENCODER_TEST_ENABLE          (0U)
#define SPEED_CONTROL_TEST_ENABLE          (0U)
#define MOTOR_ENCODER_TEST_DUTY            (40.0f)
#define MOTOR_ENCODER_TEST_START_DELAY     (CPUCLK_FREQ * 3U)
#define MOTOR_ENCODER_TEST_DEADTIME        (CPUCLK_FREQ / 2U)
#define MOTOR_ENCODER_TEST_SAMPLE_DELAY    \
    ((CPUCLK_FREQ / 1000U) * 10U)
#define MOTOR_ENCODER_TEST_SAMPLES         (200U)

/* The complete 8-channel ADC scan runs once per 10 ms control update. */
#define LINE_CONTROL_MAIN_DELAY_CYCLES \
    ((CPUCLK_FREQ / 1000U) * 10U)

/*
 * 上电后等待3秒再启动。
 */
#define LINE_CONTROL_STARTUP_DELAY_CYCLES \
    (CPUCLK_FREQ * 3U)

/*
 * 第一版实车循迹速度，单位为编码器计数/10 ms。
 * 先用6.0完成低速验证；达到20 s要求约需22计数/10 ms，
 * 必须在速度环和转向PID实车整定后再逐步提高。
 */
#define TASK_CRUISE_SPEED                  COMPETITION_CRUISE_SPEED
#define TASK_NEAR_FINISH_SPEED             (4.0f)
#define TASK_FINAL_FAR_SPEED               (4.0f)
#define TASK_FINAL_MIDDLE_SPEED            (3.0f)
#define TASK_FINAL_NEAR_SPEED              (3.0f)
#define TASK_FINISH_MARKER_CROSS_SPEED      (3.0f)

#define TASK_FINAL_MIDDLE_DISTANCE_MM      (100.0f)
#define TASK_FINAL_NEAR_DISTANCE_MM        (40.0f)
#define TASK_STOP_COMMAND_ADVANCE_MM       (10.0f)
#define TASK_STOP_CONFIRM_CYCLES           (5U)
#define TASK_LINE_LOST_ERROR_CYCLES        (150U)
#define TASK_CONTROL_PERIOD_MS             (10U)

/*
 * Curve diagnostic windows use the measured lap distance. They deliberately
 * start a little before B/D and end a little after C so entry/exit hunting is
 * included. The finish marker itself is excluded from the statistics.
 */
#define TASK_CURVE1_DEBUG_START_MM          (1350.0f)
#define TASK_CURVE1_DEBUG_END_MM            (3050.0f)
#define TASK_CURVE2_DEBUG_START_MM          (4250.0f)
#define TASK_DEBUG_SIGN_THRESHOLD           (0.12f)
#define TASK_DEBUG_CORRECTION_LIMIT         (4.75f)

typedef enum
{
    TASK_STATE_RUNNING = 0,
    TASK_STATE_FINAL_APPROACH,
    TASK_STATE_STOPPING,
    TASK_STATE_FINISHED,
    TASK_STATE_ERROR
} TaskState_t;

typedef struct
{
    uint32_t sample_count;
    uint32_t sensor_pattern_change_count;
    uint32_t position_reversal_count;
    uint32_t correction_reversal_count;
    uint32_t correction_limit_count;
    float max_abs_position;
    float max_abs_correction;
    float max_abs_target_difference;
    float max_abs_actual_difference;
    float max_abs_speed_error;
    float min_average_target;
    float max_average_target;
    uint8_t last_line_bits;
    int8_t last_position_sign;
    int8_t last_correction_sign;
} CurveDebug_t;

/*
 * Live normal-mode diagnostics.  These symbols remain available in the CCS
 * Watch window even when execution is suspended outside line_control.c.
 */
volatile LineControl_Status_t g_lineControlLive;
volatile SpeedControl_Status_t g_speedControlLive;
volatile TaskState_t g_taskState;
volatile uint32_t g_taskElapsedTimeMs;
volatile float g_taskDistanceMm;
volatile float g_taskRemainingToStopMm;
volatile uint8_t g_taskFinishMarkerConfirmed;
volatile uint8_t g_competitionTaskMode =
    (uint8_t)COMPETITION_TASK_MODE;
volatile CurveDebug_t g_curve1Debug;
volatile CurveDebug_t g_curve2Debug;
volatile uint8_t g_curveDebugActiveSegment;

static float Task_DebugAbs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int8_t Task_DebugSign(float value)
{
    if (value > TASK_DEBUG_SIGN_THRESHOLD)
    {
        return 1;
    }

    if (value < -TASK_DEBUG_SIGN_THRESHOLD)
    {
        return -1;
    }

    return 0;
}

static void Task_CurveDebugResetOne(
    volatile CurveDebug_t *debug)
{
    debug->sample_count = 0U;
    debug->sensor_pattern_change_count = 0U;
    debug->position_reversal_count = 0U;
    debug->correction_reversal_count = 0U;
    debug->correction_limit_count = 0U;
    debug->max_abs_position = 0.0f;
    debug->max_abs_correction = 0.0f;
    debug->max_abs_target_difference = 0.0f;
    debug->max_abs_actual_difference = 0.0f;
    debug->max_abs_speed_error = 0.0f;
    debug->min_average_target = 1000.0f;
    debug->max_average_target = 0.0f;
    debug->last_line_bits = 0U;
    debug->last_position_sign = 0;
    debug->last_correction_sign = 0;
}

static void Task_CurveDebugReset(void)
{
    Task_CurveDebugResetOne(&g_curve1Debug);
    Task_CurveDebugResetOne(&g_curve2Debug);
    g_curveDebugActiveSegment = 0U;
}

static void Task_CurveDebugUpdate(
    const LineControl_Status_t *line_status,
    const SpeedControl_Status_t *speed_status)
{
    volatile CurveDebug_t *debug;
    float distance_mm;
    float abs_position;
    float abs_correction;
    float average_target;
    float abs_target_difference;
    float abs_actual_difference;
    float left_speed_error;
    float right_speed_error;
    float max_speed_error;
    int8_t position_sign;
    int8_t correction_sign;

    distance_mm = TrackPosition_GetDistanceMm();

    if ((distance_mm >= TASK_CURVE1_DEBUG_START_MM) &&
        (distance_mm <= TASK_CURVE1_DEBUG_END_MM))
    {
        debug = &g_curve1Debug;
        g_curveDebugActiveSegment = 1U;
    }
    else if ((distance_mm >= TASK_CURVE2_DEBUG_START_MM) &&
             (TrackPosition_IsAtFinish() == 0U))
    {
        debug = &g_curve2Debug;
        g_curveDebugActiveSegment = 2U;
    }
    else
    {
        g_curveDebugActiveSegment = 0U;
        return;
    }

    /*
     * The transverse A marker is not a curve sample and would otherwise make
     * the sensor-pattern and correction statistics misleading.
     */
    if (line_status->wide_marker != 0U)
    {
        return;
    }

    abs_position = Task_DebugAbs(line_status->position);
    abs_correction = Task_DebugAbs(line_status->correction);
    average_target =
        (line_status->left_target + line_status->right_target) * 0.5f;
    abs_target_difference = Task_DebugAbs(
        line_status->left_target - line_status->right_target);
    abs_actual_difference = Task_DebugAbs(
        (float)speed_status->left_actual -
        (float)speed_status->right_actual);
    left_speed_error = Task_DebugAbs(
        line_status->left_target -
        (float)speed_status->left_actual);
    right_speed_error = Task_DebugAbs(
        line_status->right_target -
        (float)speed_status->right_actual);
    max_speed_error =
        (left_speed_error > right_speed_error)
            ? left_speed_error
            : right_speed_error;

    position_sign = Task_DebugSign(line_status->position);
    correction_sign = Task_DebugSign(line_status->correction);

    if (debug->sample_count != 0U)
    {
        if (debug->last_line_bits != line_status->line_bits)
        {
            debug->sensor_pattern_change_count++;
        }

        if ((position_sign != 0) &&
            (debug->last_position_sign != 0) &&
            (position_sign != debug->last_position_sign))
        {
            debug->position_reversal_count++;
        }

        if ((correction_sign != 0) &&
            (debug->last_correction_sign != 0) &&
            (correction_sign != debug->last_correction_sign))
        {
            debug->correction_reversal_count++;
        }
    }

    if (abs_position > debug->max_abs_position)
    {
        debug->max_abs_position = abs_position;
    }
    if (abs_correction > debug->max_abs_correction)
    {
        debug->max_abs_correction = abs_correction;
    }
    if (abs_target_difference > debug->max_abs_target_difference)
    {
        debug->max_abs_target_difference = abs_target_difference;
    }
    if (abs_actual_difference > debug->max_abs_actual_difference)
    {
        debug->max_abs_actual_difference = abs_actual_difference;
    }
    if (max_speed_error > debug->max_abs_speed_error)
    {
        debug->max_abs_speed_error = max_speed_error;
    }
    if (average_target < debug->min_average_target)
    {
        debug->min_average_target = average_target;
    }
    if (average_target > debug->max_average_target)
    {
        debug->max_average_target = average_target;
    }
    if (abs_correction >= TASK_DEBUG_CORRECTION_LIMIT)
    {
        debug->correction_limit_count++;
    }

    debug->last_line_bits = line_status->line_bits;
    if (position_sign != 0)
    {
        debug->last_position_sign = position_sign;
    }
    if (correction_sign != 0)
    {
        debug->last_correction_sign = correction_sign;
    }
    debug->sample_count++;
}

static void NormalMode_UpdateStatus(void)
{
    LineControl_Status_t line_status;
    SpeedControl_Status_t speed_status;

    LineControl_GetStatus(&line_status);
    SpeedControl_GetStatus(&speed_status);

    g_lineControlLive.raw_sensor = line_status.raw_sensor;
    g_lineControlLive.line_bits = line_status.line_bits;
    g_lineControlLive.active_count = line_status.active_count;
    g_lineControlLive.line_detected = line_status.line_detected;
    g_lineControlLive.wide_marker = line_status.wide_marker;
    g_lineControlLive.search_active = line_status.search_active;
    g_lineControlLive.last_direction = line_status.last_direction;
    g_lineControlLive.update_count = line_status.update_count;
    g_lineControlLive.lost_count = line_status.lost_count;
    g_lineControlLive.raw_position = line_status.raw_position;
    g_lineControlLive.position = line_status.position;
    g_lineControlLive.correction = line_status.correction;
    g_lineControlLive.running_base = line_status.running_base;
    g_lineControlLive.left_target = line_status.left_target;
    g_lineControlLive.right_target = line_status.right_target;

    g_speedControlLive.left_target = speed_status.left_target;
    g_speedControlLive.right_target = speed_status.right_target;
    g_speedControlLive.left_actual = speed_status.left_actual;
    g_speedControlLive.right_actual = speed_status.right_actual;
    g_speedControlLive.left_output = speed_status.left_output;
    g_speedControlLive.right_output = speed_status.right_output;
}

static void Task_UpdateDebug(uint32_t elapsed_time_ms)
{
    g_taskElapsedTimeMs = elapsed_time_ms;
    g_taskDistanceMm = TrackPosition_GetDistanceMm();
    g_taskRemainingToStopMm =
        TrackPosition_GetRemainingToStop();
    g_taskFinishMarkerConfirmed =
        TrackPosition_IsAtFinish();
}

static void Task_ShowReady(void)
{
    if (OLED_IsReady() == 0U)
    {
        return;
    }

    OLED_Clear();
    OLED_ShowString(20U, 2U, "READY");
    OLED_ShowString(8U, 4U, "START IN 3S");
}

static void Task_ShowRunning(void)
{
    if (OLED_IsReady() == 0U)
    {
        return;
    }

    OLED_Clear();
    OLED_ShowString(32U, 3U, "RUN");
}

static void Task_ShowFinished(uint32_t elapsed_time_ms)
{
    uint32_t seconds;
    uint32_t tenths;

    if (OLED_IsReady() == 0U)
    {
        return;
    }

    seconds = elapsed_time_ms / 1000U;
    tenths = (elapsed_time_ms % 1000U) / 100U;

    OLED_Clear();
    OLED_ShowString(20U, 1U, "FINISH");
    OLED_ShowString(8U, 3U, "TIME:");
    OLED_ShowUnsigned(44U, 3U, seconds, 2U);
    OLED_ShowString(56U, 3U, ".");
    OLED_ShowUnsigned(62U, 3U, tenths, 1U);
    OLED_ShowString(68U, 3U, "S");
    OLED_ShowString(8U, 5U, "DIST:");
    OLED_ShowUnsigned(
        44U,
        5U,
        (uint32_t)TrackPosition_GetDistanceMm(),
        4U);
    OLED_ShowString(68U, 5U, "MM");
}

static void Task_ShowError(void)
{
    if (OLED_IsReady() == 0U)
    {
        return;
    }

    OLED_Clear();
    OLED_ShowString(32U, 3U, "ERROR");
}

#if MOTOR_ENCODER_TEST_ENABLE && SPEED_CONTROL_TEST_ENABLE
#error "Enable only one diagnostic mode at a time"
#endif

#if MOTOR_ENCODER_TEST_ENABLE

enum
{
    MOTOR_TEST_PHASE_WAITING = 0U,
    MOTOR_TEST_PHASE_A_FORWARD = 1U,
    MOTOR_TEST_PHASE_A_REVERSE = 2U,
    MOTOR_TEST_PHASE_B_FORWARD = 3U,
    MOTOR_TEST_PHASE_B_REVERSE = 4U,
    MOTOR_TEST_PHASE_COMPLETE = 5U
};

typedef struct
{
    uint8_t phase;
    uint8_t complete;
    uint16_t sample;
    uint32_t run_count;
    float channel_a_command;
    float channel_b_command;
    int32_t live_left_count;
    int32_t live_right_count;
    int32_t a_forward_left_count;
    int32_t a_forward_right_count;
    int32_t a_reverse_left_count;
    int32_t a_reverse_right_count;
    int32_t b_forward_left_count;
    int32_t b_forward_right_count;
    int32_t b_reverse_left_count;
    int32_t b_reverse_right_count;
} MotorEncoderTest_Status_t;

/*
 * Add g_motorEncoderTest to the CCS Watch window.  It is deliberately global
 * and volatile so every test result remains visible after the sequence ends.
 */
volatile MotorEncoderTest_Status_t g_motorEncoderTest;

static void MotorEncoderTest_RunPhase(
    uint8_t phase,
    float channel_a_command,
    float channel_b_command,
    volatile int32_t *result_left,
    volatile int32_t *result_right)
{
    Encoder_Value_t count;
    uint16_t sample;

    Motor_Stop();
    g_motorEncoderTest.phase = MOTOR_TEST_PHASE_WAITING;
    g_motorEncoderTest.channel_a_command = 0.0f;
    g_motorEncoderTest.channel_b_command = 0.0f;
    delay_cycles(MOTOR_ENCODER_TEST_DEADTIME);

    Encoder_Reset();
    g_motorEncoderTest.live_left_count = 0;
    g_motorEncoderTest.live_right_count = 0;
    g_motorEncoderTest.sample = 0U;
    g_motorEncoderTest.phase = phase;
    g_motorEncoderTest.channel_a_command = channel_a_command;
    g_motorEncoderTest.channel_b_command = channel_b_command;

    /*
     * Motor_SetLeft controls driver channel A (PWMA/AIN1/AIN2).
     * Motor_SetRight controls driver channel B (PWMB/BIN1/BIN2).
     */
    Motor_SetBoth(channel_a_command, channel_b_command);

    for (sample = 0U; sample < MOTOR_ENCODER_TEST_SAMPLES; sample++)
    {
        delay_cycles(MOTOR_ENCODER_TEST_SAMPLE_DELAY);
        Encoder_GetCount(&count);
        g_motorEncoderTest.sample = (uint16_t)(sample + 1U);
        g_motorEncoderTest.live_left_count = count.left;
        g_motorEncoderTest.live_right_count = count.right;
    }

    Motor_Stop();
    Encoder_GetCount(&count);
    *result_left = count.left;
    *result_right = count.right;
    g_motorEncoderTest.channel_a_command = 0.0f;
    g_motorEncoderTest.channel_b_command = 0.0f;
    delay_cycles(MOTOR_ENCODER_TEST_DEADTIME);
}

static void MotorEncoderTest_Run(void)
{
    g_motorEncoderTest.phase = MOTOR_TEST_PHASE_WAITING;
    g_motorEncoderTest.complete = 0U;
    g_motorEncoderTest.sample = 0U;
    g_motorEncoderTest.run_count++;

    MotorEncoderTest_RunPhase(
        MOTOR_TEST_PHASE_A_FORWARD,
        MOTOR_ENCODER_TEST_DUTY,
        0.0f,
        &g_motorEncoderTest.a_forward_left_count,
        &g_motorEncoderTest.a_forward_right_count);

    MotorEncoderTest_RunPhase(
        MOTOR_TEST_PHASE_A_REVERSE,
        -MOTOR_ENCODER_TEST_DUTY,
        0.0f,
        &g_motorEncoderTest.a_reverse_left_count,
        &g_motorEncoderTest.a_reverse_right_count);

    MotorEncoderTest_RunPhase(
        MOTOR_TEST_PHASE_B_FORWARD,
        0.0f,
        MOTOR_ENCODER_TEST_DUTY,
        &g_motorEncoderTest.b_forward_left_count,
        &g_motorEncoderTest.b_forward_right_count);

    MotorEncoderTest_RunPhase(
        MOTOR_TEST_PHASE_B_REVERSE,
        0.0f,
        -MOTOR_ENCODER_TEST_DUTY,
        &g_motorEncoderTest.b_reverse_left_count,
        &g_motorEncoderTest.b_reverse_right_count);

    Motor_Stop();
    g_motorEncoderTest.phase = MOTOR_TEST_PHASE_COMPLETE;
    g_motorEncoderTest.complete = 1U;
}

#endif

#if SPEED_CONTROL_TEST_ENABLE

/*
 * Continuous speed-loop test status for CCS Watch.  This test bypasses the
 * gray sensor and line controller and commands both wheels at 3 counts/10 ms.
 */
volatile SpeedControl_Status_t g_speedControlTest;

static void SpeedControlTest_UpdateStatus(void)
{
    SpeedControl_Status_t status;

    SpeedControl_GetStatus(&status);
    g_speedControlTest.left_target = status.left_target;
    g_speedControlTest.right_target = status.right_target;
    g_speedControlTest.left_actual = status.left_actual;
    g_speedControlTest.right_actual = status.right_actual;
    g_speedControlTest.left_output = status.left_output;
    g_speedControlTest.right_output = status.right_output;
}

#endif

int main(void)

{
    /*
     * 初始化SysConfig生成的所有外设。
     */
    SYSCFG_DL_init();
    StepperMotor_Init();
    StepperEncoder_Init();

#if MOTOR_ENCODER_TEST_ENABLE
    /*
     * Do not initialize SpeedControl in this mode: its 10 ms interrupt would
     * overwrite the direct motor commands used by the bench test.
     */
    Motor_Init();
    Encoder_Init();

    delay_cycles(MOTOR_ENCODER_TEST_START_DELAY);
    MotorEncoderTest_Run();

    while (1)
    {
        Motor_Stop();
    }
#elif SPEED_CONTROL_TEST_ENABLE
    Motor_Init();
    Encoder_Init();
    SpeedControl_Init();

    delay_cycles(LINE_CONTROL_STARTUP_DELAY_CYCLES);
    SpeedControl_SetTarget(3.0f, 3.0f);

    while (1)
    {
        SpeedControlTest_UpdateStatus();
        delay_cycles(LINE_CONTROL_MAIN_DELAY_CYCLES);
    }
#else
    {
        uint32_t start_tick;
        uint32_t last_control_tick;
        uint32_t current_tick;
        uint32_t elapsed_time_ms;
        uint8_t stop_confirm_cycles = 0U;
        LineControl_Status_t line_status;
        SpeedControl_Status_t speed_status;

        /*
         * 初始化顺序不能随意改变。
         */
        Motor_Init();
        Encoder_Init();
        GraySensor_Init();
        K230Uart_Init();
        SpeedControl_Init();
        LineControl_Init();
        TrackPosition_Init();
        OLED_Init();

        g_taskState = TASK_STATE_RUNNING;
        Task_UpdateDebug(0U);
        Task_ShowReady();

        /*
         * 按键暂不接入：上电后等待3秒自动开始。
         */
        delay_cycles(
            LINE_CONTROL_STARTUP_DELAY_CYCLES);

        /*
         * 正式起跑前同时清零车轮编码器、里程和比赛计时。
         */
        Encoder_Reset();
        TrackPosition_Reset();
        Task_CurveDebugReset();
        LineControl_SetBaseSpeed(TASK_CRUISE_SPEED);

        start_tick = SpeedControl_GetTickCount();
        last_control_tick = start_tick;
        elapsed_time_ms = 0U;
        Task_ShowRunning();

        while (1)
        {
            K230Uart_Poll();

            current_tick = 
            SpeedControl_GetTickCount();

            /*
             * 只在新的10 ms硬件节拍到来时更新一次循迹。
             * 速度环本身仍在TIMER_CONTROL中断中独立运行。
             */
            if (current_tick == last_control_tick)
            {
                continue;
            }

            last_control_tick = current_tick;
            if ((g_taskState != TASK_STATE_FINISHED) &&
                (g_taskState != TASK_STATE_ERROR))
            {
                elapsed_time_ms =
                    (current_tick - start_tick) *
                    TASK_CONTROL_PERIOD_MS;
            }

            if ((g_taskState == TASK_STATE_RUNNING) ||
                (g_taskState == TASK_STATE_FINAL_APPROACH))
            {
                /*
                 * 普通循迹和终点168 mm定位阶段都继续跟随中心黑线。
                 */
                LineControl_Update();
                LineControl_GetStatus(&line_status);

                TrackPosition_Update(
                    line_status.wide_marker,
                    TASK_CONTROL_PERIOD_MS);
                SpeedControl_GetStatus(&speed_status);
                Task_CurveDebugUpdate(
                    &line_status,
                    &speed_status);

                if (line_status.lost_count >=
                    TASK_LINE_LOST_ERROR_CYCLES)
                {
                    LineControl_Stop();
                    g_taskState = TASK_STATE_ERROR;
                    Task_ShowError();
                }
                else if (g_taskState == TASK_STATE_RUNNING)
                {
                    if (TrackPosition_IsAtFinish() != 0U)
                    {
                        /*
                         * 宽线已连续确认。停车目标是首次检测里程+168 mm。
                         */
                        g_taskState =
                            TASK_STATE_FINAL_APPROACH;
                    }
                    else if (
                        (TrackPosition_IsFinishDetectionEnabled() != 0U) &&
                        (line_status.wide_marker != 0U))
                    {
                        /*
                         * The transverse A marker is not an ordinary line
                         * position sample. While confirming it, cross it at a
                         * low equal wheel speed instead of allowing the line
                         * PID to pivot on an asymmetric multi-probe pattern.
                         */
                        SpeedControl_SetTarget(
                            TASK_FINISH_MARKER_CROSS_SPEED,
                            TASK_FINISH_MARKER_CROSS_SPEED);
                    }
                    else if (TrackPosition_IsNearFinish() != 0U)
                    {
                        /*
                         * 理论终点前400 mm降速，保证横线有足够采样次数。
                         */
                        LineControl_SetBaseSpeed(
                            TASK_NEAR_FINISH_SPEED);
                    }
                    else
                    {
                        LineControl_SetBaseSpeed(
                            TASK_CRUISE_SPEED);
                    }
                }

                if (g_taskState ==
                    TASK_STATE_FINAL_APPROACH)
                {
                    float remaining_mm =
                        TrackPosition_GetRemainingToStop();

                    if (remaining_mm <=
                        TASK_STOP_COMMAND_ADVANCE_MM)
                    {
                        /*
                         * 在目标前约5 mm发出停止命令，补偿低速制动余量。
                         * 该值需要根据实车最终微调。
                         */
                        LineControl_Stop();
                        stop_confirm_cycles = 0U;
                        g_taskState = TASK_STATE_STOPPING;
                    }
                    else if (remaining_mm <=
                             TASK_FINAL_NEAR_DISTANCE_MM)
                    {
                        LineControl_SetBaseSpeed(
                            TASK_FINAL_NEAR_SPEED);
                    }
                    else if (remaining_mm <=
                             TASK_FINAL_MIDDLE_DISTANCE_MM)
                    {
                        LineControl_SetBaseSpeed(
                            TASK_FINAL_MIDDLE_SPEED);
                    }
                    else
                    {
                        LineControl_SetBaseSpeed(
                            TASK_FINAL_FAR_SPEED);
                    }
                }
            }
            else if (g_taskState == TASK_STATE_STOPPING)
            {
                /*
                 * 停止命令后继续累计编码器，直到左右轮连续50 ms为零。
                 */
                SpeedControl_Stop();
                TrackPosition_Update(
                    0U,
                    TASK_CONTROL_PERIOD_MS);
                SpeedControl_GetStatus(&speed_status);

                if ((speed_status.left_actual == 0) &&
                    (speed_status.right_actual == 0))
                {
                    if (stop_confirm_cycles <
                        TASK_STOP_CONFIRM_CYCLES)
                    {
                        stop_confirm_cycles++;
                    }
                }
                else
                {
                    stop_confirm_cycles = 0U;
                }

                if (stop_confirm_cycles >=
                    TASK_STOP_CONFIRM_CYCLES)
                {
                    g_taskState = TASK_STATE_FINISHED;
                    Task_UpdateDebug(elapsed_time_ms);
                    Task_ShowFinished(elapsed_time_ms);
                }
            }
            else
            {
                /*
                 * 完成或故障后保持电机关闭。
                 */
                SpeedControl_Stop();
            }

            NormalMode_UpdateStatus();
            Task_UpdateDebug(elapsed_time_ms);

            /*
             * 球杆编码器和K230接收保持更新，不影响本题循迹状态机。
             */
            K230Uart_Poll();
            StepperEncoder_Update();
        }
    }

#endif
}
