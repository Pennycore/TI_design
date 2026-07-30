#ifndef GRAY_SENSOR_H_
#define GRAY_SENSOR_H_

#include <stdint.h>

#define GRAY_SENSOR_CHANNEL_COUNT    (8U)

/*
 * Live calibration/debug values:
 * - g_graySensorRaw[]: latest 12-bit ADC result for CH1...CH8.
 * - g_graySensorWhite[]: measured white values at the final mounting height.
 * - g_graySensorBlack[]: measured black values at the final mounting height.
 *
 * The calibration arrays must be replaced after measuring the actual sensor.
 */
extern volatile uint16_t
    g_graySensorRaw[GRAY_SENSOR_CHANNEL_COUNT];
extern volatile uint16_t
    g_graySensorWhite[GRAY_SENSOR_CHANNEL_COUNT];
extern volatile uint16_t
    g_graySensorBlack[GRAY_SENSOR_CHANNEL_COUNT];
extern volatile uint32_t g_graySensorAdcTimeoutCount;

/*
 * Initialize the 74HC4051 address lines and ADC state.
 */
void GraySensor_Init(void);

/*
 * Select CH1...CH8 through AD0/AD1/AD2, sample OUT with ADC0 channel 0, and
 * apply per-channel hysteresis. bit0...bit7 correspond to CH1...CH8.
 * A white surface is returned as 1 and a black surface as 0 so the existing
 * black-line tracking layer remains compatible.
 */
uint8_t GraySensor_Read(void);

#endif /* GRAY_SENSOR_H_ */
