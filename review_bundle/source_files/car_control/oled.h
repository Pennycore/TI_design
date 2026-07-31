#ifndef OLED_H_
#define OLED_H_

#include <stdint.h>

void OLED_Init(void);
uint8_t OLED_IsReady(void);
void OLED_Clear(void);

/*
 * 在指定位置显示字符串。
 * column：0～127
 * page：0～7，每一页高度为8像素
 */
void OLED_ShowString(
    uint8_t column,
    uint8_t page,
    const char *text);

/*
 * 显示固定宽度无符号整数。
 * width表示显示位数，不足的高位补0。
 */
void OLED_ShowUnsigned(
    uint8_t column,
    uint8_t page,
    uint32_t value,
    uint8_t width);

/*
 * 显示固定宽度有符号整数。
 * width包含符号位。
 */
void OLED_ShowSigned(
    uint8_t column,
    uint8_t page,
    int32_t value,
    uint8_t width);

/*
 * 清除指定的一行。
 */
void OLED_ClearLine(uint8_t page);

/*
 * OLED单独测试界面。
 */
void OLED_ShowTest(void);

#endif /* OLED_H_ */
