#ifndef PID_H_
#define PID_H_

typedef struct
{
    /* PID三个参数 */
    float kp;
    float ki;
    float kd;

    /* PID计算周期，单位：秒 */
    float dt;

    /* 历史数据 */
    float integral;
    float last_error;

    /* 输出限幅 */
    float output_min;
    float output_max;

    /* 积分限幅 */
    float integral_min;
    float integral_max;

} PID_t;

/*
 * 初始化PID。
 */
void PID_Init(
    PID_t *pid,
    float kp,
    float ki,
    float kd,
    float dt,
    float output_min,
    float output_max
);

/*
 * 重新清空PID历史值。
 */
void PID_Reset(PID_t *pid);

/*
 * 完成一次PID运算。
 *
 * target：目标值
 * actual：实际值
 *
 * 返回：PID输出
 */
float PID_Calculate(
    PID_t *pid,
    float target,
    float actual
);

#endif /* PID_H_ */