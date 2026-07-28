#ifndef LINE_CONTROL_H_
#define LINE_CONTROL_H_

#include <stdint.h>

/*
 * 循迹控制周期约为 10 ms。
 * GraySensor_Read() 内部使用约 1 ms 完成帧同步，main 再等待约 9 ms。
 */
#define LINE_CONTROL_PERIOD_SECONDS       (0.010f)

/*
 * 目标速度单位与 SpeedControl 相同：
 * 编码器四倍频计数/10 ms。
 *
 * 首次实车测试使用较低速度，确认方向正确后再逐步提高。
 */
#define LINE_CONTROL_DEFAULT_BASE_SPEED   (3.0f)
#define LINE_CONTROL_MAX_WHEEL_SPEED      (6.0f)
#define LINE_CONTROL_MAX_CORRECTION       (3.0f)

/*
 * 循迹 PID 初始值。
 * 先调整 KP，再加入少量 KD；一般暂时不需要 KI。
 */
#define LINE_CONTROL_KP                   (4.0f)
#define LINE_CONTROL_KI                   (0.0f)
#define LINE_CONTROL_KD                   (0.025f)

/*
 * 传感器安装方向配置。
 * 如果实测 bit0 位于车身右侧，将其改为 0。
 */
#define LINE_CONTROL_BIT0_IS_LEFT         (1U)

/*
 * 当前按照“白底黑线”配置：传感器读到黑线时对应 bit 为 0。
 * 若以后改为循白线，将其改为 0。
 */
#define LINE_CONTROL_TRACK_BLACK_LINE     (1U)

typedef struct
{
    uint8_t raw_sensor;
    uint8_t line_bits;
    uint8_t active_count;
    uint8_t line_detected;
    uint32_t update_count;
    uint32_t lost_count;
    float position;
    float correction;
    float left_target;
    float right_target;
} LineControl_Status_t;

/*
 * 初始化循迹 PID 和状态。
 * 应在 GraySensor_Init()、SpeedControl_Init() 之后调用。
 */
void LineControl_Init(void);

/*
 * 读取一次八路灰度数据并更新左右轮目标速度。
 * 未检测到黑线时默认立即停车。
 */
void LineControl_Update(void);

/*
 * 停止循迹并清空循迹 PID。
 */
void LineControl_Stop(void);

/*
 * 修改直线基础速度。
 */
void LineControl_SetBaseSpeed(float base_speed);

/*
 * 修改循迹 PID 参数。
 */
void LineControl_SetTunings(float kp, float ki, float kd);

/*
 * 获取最近一次循迹状态，便于 CCS Watch 调试。
 */
void LineControl_GetStatus(LineControl_Status_t *status);

#endif /* LINE_CONTROL_H_ */
