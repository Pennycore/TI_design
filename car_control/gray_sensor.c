#include "gray_sensor.h"
#include "ti_msp_dl_config.h"

#define GRAY_SENSOR_CLK_HIGH_CYCLES \
    ((CPUCLK_FREQ / 1000000U) * 5U)
#define GRAY_SENSOR_FRAME_SYNC_CYCLES \
    (CPUCLK_FREQ / 1000U)

void GraySensor_Init(void)
{
    /* 手册示例要求 CLK 初始化为低电平。 */
    DL_GPIO_clearPins(GPIO_GRAY_PORT, GPIO_GRAY_CLK_PIN);
}

uint8_t GraySensor_Read(void)
{
    uint8_t data = 0U;
    uint8_t i;

    /*
     * CLK 超过约 0.8 ms 无上升沿时，传感器会回到第 1 路。
     * 按手册建议等待 1 ms，确保每帧都从 bit0 开始。
     */
    delay_cycles(GRAY_SENSOR_FRAME_SYNC_CYCLES);

    for (i = 0U; i < 8U; i++) {
        /* 高电平写数据、低电平读数据。 */
        DL_GPIO_clearPins(GPIO_GRAY_PORT, GPIO_GRAY_CLK_PIN);

        if (DL_GPIO_readPins(GPIO_GRAY_PORT, GPIO_GRAY_DAT_PIN) != 0U) {
            data |= (uint8_t)(1U << i);
        }

        /*
         * 上升沿使传感器准备下一位；高电平必须保持至少 5 us。
         * 单帧内相邻时钟的间隔必须远小于 1 ms。
         */
        DL_GPIO_setPins(GPIO_GRAY_PORT, GPIO_GRAY_CLK_PIN);
        delay_cycles(GRAY_SENSOR_CLK_HIGH_CYCLES);
    }

    return data;
}
