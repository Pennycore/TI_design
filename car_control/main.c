#include "ti_msp_dl_config.h"

#include "encoder.h"
#include "gray_sensor.h"
#include "k230_uart.h"
#include "line_control.h"
#include "motor.h"
#include "speed_control.h"

/*
<<<<<<< HEAD
 * Temporary bench-test mode.
 *
 * IMPORTANT: lift both wheels off the ground before flashing this build.
 * Set to 0 after motor/encoder direction testing is complete.
=======
 * GraySensor_Read()内部约等待1ms。
 * 主循环额外等待约9ms，使循迹周期接近10ms。
>>>>>>> c4c408127b779efd2b8ce6b4ef4b3f90f99e0207
 */
#define MOTOR_ENCODER_TEST_ENABLE          (0U)
#define SPEED_CONTROL_TEST_ENABLE          (0U)
#define MOTOR_ENCODER_TEST_DUTY            (40.0f)
#define MOTOR_ENCODER_TEST_START_DELAY     (CPUCLK_FREQ * 3U)
#define MOTOR_ENCODER_TEST_DEADTIME        (CPUCLK_FREQ / 2U)
#define MOTOR_ENCODER_TEST_SAMPLE_DELAY    \
    ((CPUCLK_FREQ / 1000U) * 10U)
#define MOTOR_ENCODER_TEST_SAMPLES         (200U)

/* GraySensor_Read() takes less than 0.1 ms; wait about 10 ms per update. */
#define LINE_CONTROL_MAIN_DELAY_CYCLES \
    ((CPUCLK_FREQ / 1000U) * 10U)

/*
 * 上电后等待3秒再启动。
 */
#define LINE_CONTROL_STARTUP_DELAY_CYCLES \
    (CPUCLK_FREQ * 3U)

/*
 * Live normal-mode diagnostics.  These symbols remain available in the CCS
 * Watch window even when execution is suspended outside line_control.c.
 */
volatile LineControl_Status_t g_lineControlLive;
volatile SpeedControl_Status_t g_speedControlLive;

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
    g_lineControlLive.search_active = line_status.search_active;
    g_lineControlLive.last_direction = line_status.last_direction;
    g_lineControlLive.update_count = line_status.update_count;
    g_lineControlLive.lost_count = line_status.lost_count;
    g_lineControlLive.raw_position = line_status.raw_position;
    g_lineControlLive.position = line_status.position;
    g_lineControlLive.correction = line_status.correction;
    g_lineControlLive.left_target = line_status.left_target;
    g_lineControlLive.right_target = line_status.right_target;

    g_speedControlLive.left_target = speed_status.left_target;
    g_speedControlLive.right_target = speed_status.right_target;
    g_speedControlLive.left_actual = speed_status.left_actual;
    g_speedControlLive.right_actual = speed_status.right_actual;
    g_speedControlLive.left_output = speed_status.left_output;
    g_speedControlLive.right_output = speed_status.right_output;
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
    /*
     * 初始化顺序不能随意改变。
     */
    Motor_Init();
    Encoder_Init();
    GraySensor_Init();
    K230Uart_Init();
    SpeedControl_Init();
    LineControl_Init();

    /*
     * 等待3秒，方便将小车放到黑线上并移开双手。
     */
    delay_cycles(
        LINE_CONTROL_STARTUP_DELAY_CYCLES);

    while (1)
    {
        K230Uart_Poll();

        /*
         * 读取灰度传感器并更新循迹控制。
         */
        LineControl_Update();
<<<<<<< HEAD
        NormalMode_UpdateStatus();
        delay_cycles(LINE_CONTROL_MAIN_DELAY_CYCLES);
    }
#endif
}
=======

        K230Uart_Poll();

        delay_cycles(
            LINE_CONTROL_MAIN_DELAY_CYCLES);
    }
}
>>>>>>> c4c408127b779efd2b8ce6b4ef4b3f90f99e0207
