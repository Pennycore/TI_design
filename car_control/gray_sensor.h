#ifndef GRAY_SENSOR_H_
#define GRAY_SENSOR_H_

#include <stdint.h>

/*
 * Initialize the Ganv eight-channel gray sensor serial interface.
 * CLK is a push-pull output and idles high; DAT is a pull-up input.
 */
void GraySensor_Init(void);

/*
 * Read eight digital channels through CLK and DAT.
 *
 * bit0...bit7 correspond to probes 1...8.
 * After module calibration, white returns 1 and black returns 0.
 */
uint8_t GraySensor_Read(void);

#endif /* GRAY_SENSOR_H_ */
