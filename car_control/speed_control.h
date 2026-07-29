#ifndef SPEED_CONTROL_H_
#define SPEED_CONTROL_H_

#include <stdint.h>

/*
 * 控制周期由 TIMER_CONTROL 决定，当前 SysConfig 配置为 10 ms。
 * 目标速度和实际速度的单位都是“编码器四倍频计数/10 ms”。
 *
 * 以下 PID 参数只是低速起步值，必须根据实车响应继续调整。
 */
#define SPEED_CONTROL_PERIOD_SECONDS    (0.01f)

#define SPEED_CONTROL_LEFT_KP           (2.0f)
#define SPEED_CONTROL_LEFT_KI           (4.0f)
#define SPEED_CONTROL_LEFT_KD           (0.0f)

#define SPEED_CONTROL_RIGHT_KP          (2.0f)
#define SPEED_CONTROL_RIGHT_KI          (4.0f)
#define SPEED_CONTROL_RIGHT_KD          (0.0f)

/*
 * 克服 32 cm x 24 cm 大车体的静摩擦和负载需要一个基础占空比。
 * PID 只在此前馈值附近修正速度大小，电机方向始终由目标速度符号决定。
 */
#define SPEED_CONTROL_MIN_DRIVE_DUTY    (35.0f)
#define SPEED_CONTROL_FEEDFORWARD_GAIN  (6.0f)

/*
 * The wheels turned correctly with the chassis lifted, but the original
 * command could not reliably overcome the complete car's static friction.
 * While an encoder still reports zero speed, use a short breakaway-level
 * command.  Closed-loop control automatically takes over once motion starts.
 */
#define SPEED_CONTROL_BREAKAWAY_DUTY     (60.0f)
#define SPEED_CONTROL_MOVING_THRESHOLD  (1.0f)

typedef struct
{
    float left_target;
    float right_target;
    int32_t left_actual;
    int32_t right_actual;
    float left_output;
    float right_output;
} SpeedControl_Status_t;

/*
 * 初始化左右轮 PID，并启动 10 ms 控制定时器。
 * 调用顺序应为：
 * SYSCFG_DL_init -> Motor_Init -> Encoder_Init -> SpeedControl_Init
 */
void SpeedControl_Init(void);

/*
 * 设置左右轮目标速度，单位为编码器四倍频计数/10 ms。
 * 正数为正转，负数为反转，0 为停止。
 */
void SpeedControl_SetTarget(float left_target, float right_target);

/*
 * 立即将目标速度和电机输出清零。
 */
void SpeedControl_Stop(void);

/*
 * 执行一次双轮速度闭环，由 10 ms 定时器中断自动调用。
 */
void SpeedControl_Update(void);

/*
 * 读取最近一次控制状态，便于 CCS Watch 或串口调试。
 */
void SpeedControl_GetStatus(SpeedControl_Status_t *status);

/*
 * 运行时修改左右轮 PID 参数。修改后会清空 PID 历史量。
 */
void SpeedControl_SetTunings(
    float left_kp,
    float left_ki,
    float left_kd,
    float right_kp,
    float right_ki,
    float right_kd);

#endif /* SPEED_CONTROL_H_ */
