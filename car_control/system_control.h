#ifndef SYSTEM_CONTROL_H_
#define SYSTEM_CONTROL_H_

#include <stdint.h>

/*
 * 整车任务状态。
 */
typedef enum
{
    SYSTEM_STATE_WAIT_START = 0, /* 等待启动 */
    SYSTEM_STATE_LEAVE_A,        /* 离开A点 */
    SYSTEM_STATE_RUN_TO_B,       /* 从A点前往B点 */
    SYSTEM_STATE_PASS_B,         /* 已识别到B点 */
    SYSTEM_STATE_RETURN_TO_A,    /* 从B点返回A点 */
    SYSTEM_STATE_BRAKING,        /* 到达A点，正在停车 */
    SYSTEM_STATE_FINISHED,       /* 任务完成 */
    SYSTEM_STATE_ERROR           /* 故障 */
} SystemState;

/*
 * 故障类型。
 */
typedef enum
{
    SYSTEM_ERROR_NONE = 0,
    SYSTEM_ERROR_LINE_LOST,      /* 长时间丢线 */
    SYSTEM_ERROR_TIMEOUT,        /* 任务运行超时 */
    SYSTEM_ERROR_SENSOR          /* 传感器故障 */
} SystemError;

/*
 * 状态机输入。
 *
 * 所有输入均应当是经过处理后的逻辑信号。
 * start_pressed 最好只在按键按下沿时置1一个周期。
 * marker_a 和 marker_b 可以保持为1，状态机内部会进行边沿检测。
 */
typedef struct
{
    uint8_t start_pressed;
    uint8_t marker_a;
    uint8_t marker_b;
    uint8_t line_lost;
    uint8_t sensor_error;
} SystemControlInput;

/*
 * 初始化整车任务状态机。
 */
void SystemControl_Init(void);

/*
 * 复位状态机，返回等待启动状态。
 */
void SystemControl_Reset(void);

/*
 * 更新状态机。
 *
 * input：当前输入信号。
 * delta_ms：距离上一次调用经过的时间，建议固定传入10。
 */
void SystemControl_Update(const SystemControlInput *input,
                          uint32_t delta_ms);

/*
 * 获取当前状态。
 */
SystemState SystemControl_GetState(void);

/*
 * 获取故障类型。
 */
SystemError SystemControl_GetError(void);

/*
 * 获取从启动开始到现在的运行时间，单位ms。
 */
uint32_t SystemControl_GetElapsedTimeMs(void);

/*
 * 获取第一次经过B点的时间，单位ms。
 * 尚未经过B点时返回0。
 */
uint32_t SystemControl_GetBTimeMs(void);

/*
 * 获取完成一圈的最终时间，单位ms。
 * 尚未完成时返回0。
 */
uint32_t SystemControl_GetFinishTimeMs(void);

/*
 * 当前是否允许小车循迹行驶。
 */
uint8_t SystemControl_ShouldRun(void);

/*
 * 当前是否要求小车减速停车。
 */
uint8_t SystemControl_ShouldBrake(void);

/*
 * 当前是否已经完成任务。
 */
uint8_t SystemControl_IsFinished(void);

/*
 * 当前是否发生故障。
 */
uint8_t SystemControl_HasError(void);

/*
 * 获取用于OLED显示的简短状态字符串。
 */
const char *SystemControl_GetStateText(void);

#endif