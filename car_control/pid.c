#include "pid.h"

static float PID_Limit(
    float value,
    float minimum,
    float maximum
)
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

void PID_Init(
    PID_t *pid,
    float kp,
    float ki,
    float kd,
    float dt,
    float output_min,
    float output_max
)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->dt = dt;

    pid->integral   = 0.0f;
    pid->last_error = 0.0f;

    pid->output_min = output_min;
    pid->output_max = output_max;

    /*
     * 积分项先限制在较小范围内，
     * 防止积分持续累积。
     */
    pid->integral_min = -50.0f;
    pid->integral_max = 50.0f;
}

void PID_Reset(PID_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->integral   = 0.0f;
    pid->last_error = 0.0f;
}

float PID_Calculate(
    PID_t *pid,
    float target,
    float actual
)
{
    float error;
    float derivative;
    float output;

    if ((pid == 0) || (pid->dt <= 0.0f))
    {
        return 0.0f;
    }

    /*
     * 当前误差：
     * 目标值减去实际值。
     */
    error = target - actual;

    /*
     * 积分项：
     * 对误差进行累积。
     */
    pid->integral += error * pid->dt;

    pid->integral = PID_Limit(
        pid->integral,
        pid->integral_min,
        pid->integral_max
    );

    /*
     * 微分项：
     * 误差变化速度。
     */
    derivative =
        (error - pid->last_error) / pid->dt;

    /*
     * 位置式PID公式。
     */
    output =
        pid->kp * error +
        pid->ki * pid->integral +
        pid->kd * derivative;

    /*
     * 将输出限制在电机允许范围内。
     */
    output = PID_Limit(
        output,
        pid->output_min,
        pid->output_max
    );

    pid->last_error = error;

    return output;
}