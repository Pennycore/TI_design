#ifndef STEPPER_ENCODER_H
#define STEPPER_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

/*
 * MS42CG encoder interface:
 *   A   -> PA1 / TIMG8_CCP0
 *   B   -> PA0 / TIMG8_CCP1
 *   PWM -> PB20 / TIMG12_CCP0
 *   Z   -> PA25
 *
 * The manual specifies 1000 AB lines per revolution. TIMG8 quadrature mode
 * counts all four edges, therefore one mechanical revolution is 4000 counts.
 */
#define STEPPER_ENCODER_COUNTS_PER_REVOLUTION    (4000)

void StepperEncoder_Init(void);
void StepperEncoder_Update(void);
void StepperEncoder_Reset(void);
int32_t StepperEncoder_GetCount(void);
float StepperEncoder_GetAngleDegrees(void);
uint32_t StepperEncoder_GetIndexCount(void);
bool StepperEncoder_GetPwmAngleDegrees(float *angle_degrees);

#endif
