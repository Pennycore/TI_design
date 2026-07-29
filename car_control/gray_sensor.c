#include "gray_sensor.h"
#include "ti_msp_dl_config.h"

#define GRAY_SENSOR_CLK_LOW_CYCLES \
    ((CPUCLK_FREQ / 1000000U) * 2U)
#define GRAY_SENSOR_CLK_HIGH_CYCLES \
    ((CPUCLK_FREQ / 1000000U) * 5U)

void GraySensor_Init(void)
{
    /*
     * The Ganv serial example keeps CLK high while idle.  A falling edge
     * presents the current channel on DAT; the following rising edge advances
     * the sensor to the next channel.
     */
    DL_GPIO_setPins(GPIO_GRAY_PORT, GPIO_GRAY_CLK_PIN);
}

uint8_t GraySensor_Read(void)
{
    uint8_t data = 0U;
    uint8_t i;

    for (i = 0U; i < 8U; i++)
    {
        /* Falling edge: make the current channel available on DAT. */
        DL_GPIO_clearPins(GPIO_GRAY_PORT, GPIO_GRAY_CLK_PIN);
        delay_cycles(GRAY_SENSOR_CLK_LOW_CYCLES);

        if (DL_GPIO_readPins(GPIO_GRAY_PORT, GPIO_GRAY_DAT_PIN) != 0U)
        {
            data |= (uint8_t)(1U << i);
        }

        /*
         * Rising edge advances to the next channel.  Keep the high level for
         * about 5 us, matching the manufacturer's MSPM0 example.
         */
        DL_GPIO_setPins(GPIO_GRAY_PORT, GPIO_GRAY_CLK_PIN);
        delay_cycles(GRAY_SENSOR_CLK_HIGH_CYCLES);
    }

    return data;
}
