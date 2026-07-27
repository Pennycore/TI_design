#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

/*
 * 编码器方向修正系数，只能取 1 或 -1。
 *
 * 若车辆向前行驶时某一侧计数为负，可将对应值改成 -1；
 * 也可以交换该侧编码器的 A、B 接线。
 */
#define ENCODER_LEFT_DIRECTION     (1)
#define ENCODER_RIGHT_DIRECTION    (1)

typedef struct
{
    int32_t left;
    int32_t right;
} Encoder_Value_t;

/*
 * 初始化编码器状态和 GPIO 中断。
 * 必须在 SYSCFG_DL_init() 之后调用。
 */
void Encoder_Init(void);

/*
 * 清零累计计数和速度。
 */
void Encoder_Reset(void);

/*
 * 读取左右编码器的累计计数。
 * 当前使用四倍频解码，因此一个完整 AB 周期产生 4 个计数。
 */
void Encoder_GetCount(Encoder_Value_t *count);

/*
 * 读取最近一个 10 ms 周期内的增量。
 *
 * 换算关系：
 * counts_per_second = speed * 100
 * rpm = speed * 6000 / 每圈四倍频计数
 */
void Encoder_GetSpeed(Encoder_Value_t *speed);

/*
 * 锁存一次速度。通常由 10 ms 定时器中断调用。
 */
void Encoder_Sample(void);

#endif /* ENCODER_H_ */
