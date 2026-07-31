#ifndef TRACK_POSITION_H_
#define TRACK_POSITION_H_

#include <stdint.h>

/*
 * ============================================================
 * 小车里程标定参数
 * ============================================================
 */

/*
 * 小车驱动轮实际直径，单位mm。
 * 用户实测为65 mm。
 */
#define TRACK_WHEEL_DIAMETER_MM               (65.0f)

/*
 * 编码器每个车轮转一圈的实测计数。
 *
 * 实测结果：
 * 左轮10圈：14672，单圈1467.2
 * 右轮10圈：14397，单圈1439.7
 */
#define TRACK_LEFT_COUNTS_PER_WHEEL_REV       (1467.2f)
#define TRACK_RIGHT_COUNTS_PER_WHEEL_REV      (1439.7f)

/*
 * 赛道中心线由两段1.5 m直线和两个r=0.5 m半圆组成。
 *
 * A->B = 1500 mm
 * A->C = 1500 + pi*500 = 3070.8 mm
 * A->D = 3000 + pi*500 = 4570.8 mm
 * 一圈  = 3000 + 2*pi*500 = 6141.6 mm
 *
 * 这些是图纸理论值；最终仍需通过实车修正轮胎打滑造成的误差。
 */
#define TRACK_DISTANCE_TO_B_MM                (1500.0f)
#define TRACK_DISTANCE_TO_C_MM                (3070.8f)
#define TRACK_DISTANCE_TO_D_MM                (4570.8f)
#define TRACK_LAP_DISTANCE_MM                 (6141.6f)

/*
 * 8路传感器探头中心线位于停车参考点前方168 mm。
 * 终点横线第一次被检测到后，小车参考点还需前进该距离。
 */
#define TRACK_SENSOR_TO_REFERENCE_MM          (168.0f)

/*
 * 距离终点还有多少毫米时开始减速。
 */
#define TRACK_FINISH_SLOWDOWN_ADVANCE_MM      (400.0f)

/*
 * 行驶到完整一圈距离的85%后，
 * 才允许识别A点终点横线。
 */
#define TRACK_FINISH_ENABLE_RATIO             (0.92f)

/*
 * A点横线连续检测有效时间。
 */
#define TRACK_FINISH_MARKER_CONFIRM_MS        (50U)

/*
 * 判断到达B、C、D点时允许的距离误差。
 */
#define TRACK_POINT_TOLERANCE_MM              (100.0f)

typedef enum
{
    TRACK_POINT_A_START = 0,
    TRACK_POINT_B,
    TRACK_POINT_C,
    TRACK_POINT_D,
    TRACK_POINT_A_FINISH
} TrackPoint;

/*
 * 必须在Encoder_Init()之后调用。
 */
void TrackPosition_Init(void);

/*
 * 开始一次新任务时清零里程。
 */
void TrackPosition_Reset(void);

/*
 * 更新小车累计里程。
 *
 * marker_a：
 * 1 = 检测到A点终点横线
 * 0 = 未检测到
 *
 * delta_ms：
 * 本次更新与上一次更新的时间间隔。
 */
void TrackPosition_Update(uint8_t marker_a, uint32_t delta_ms);

/*
 * 获取累计里程，单位mm。
 */
float TrackPosition_GetDistanceMm(void);
float TrackPosition_GetLeftDistanceMm(void);
float TrackPosition_GetRightDistanceMm(void);

/*
 * 点位判断。
 */
uint8_t TrackPosition_HasReachedB(void);
uint8_t TrackPosition_HasReachedC(void);
uint8_t TrackPosition_HasReachedD(void);

/*
 * 终点判断。
 */
uint8_t TrackPosition_IsNearFinish(void);
uint8_t TrackPosition_IsFinishDetectionEnabled(void);
uint8_t TrackPosition_IsAtFinish(void);

/*
 * A点横线确认后的停车定位。
 */
float TrackPosition_GetFinishMarkerDistanceMm(void);
float TrackPosition_GetStopTargetDistanceMm(void);
float TrackPosition_GetRemainingToStop(void);
uint8_t TrackPosition_IsStopTargetReached(void);

/*
 * 获取当前点位。
 */
TrackPoint TrackPosition_GetCurrentPoint(void);

/*
 * 获取剩余距离，单位mm。
 */
float TrackPosition_GetRemainingToB(void);
float TrackPosition_GetRemainingToFinish(void);

#endif /* TRACK_POSITION_H_ */
