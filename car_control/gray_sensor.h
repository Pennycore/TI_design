#ifndef GRAY_SENSOR_H_
#define GRAY_SENSOR_H_

#include <stdint.h>

/*
 * 初始化感为八路灰度传感器的串行通信引脚。
 * CLK 为推挽输出并保持低电平，DAT 为上拉输入。
 */
void GraySensor_Init(void);

/*
 * 通过 CLK、DAT 读取八路数字量状态。
 *
 * bit0～bit7 依次对应第 1～8 路探头。
 * 校准后，接近白场输出 1，接近黑场输出 0。
 * 函数会先等待 1 ms 完成帧同步，随后读取一帧数据。
 */
uint8_t GraySensor_Read(void);

#endif /* GRAY_SENSOR_H_ */
