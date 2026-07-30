#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    STEPPER_DIRECTION_NEGATIVE = 0,
    STEPPER_DIRECTION_POSITIVE = 1
} StepperMotor_Direction_t;

/*
 * D36A control interface:
 *   STEP1 -> PA23 / TIMG7_CCP0
 *   DIR1  -> PA13
 *   EN1   -> PA12 (active high)
 */
void StepperMotor_Init(void);
bool StepperMotor_StartSteps(
    uint32_t steps,
    uint32_t frequency_hz,
    StepperMotor_Direction_t direction);
bool StepperMotor_StartContinuous(
    uint32_t frequency_hz,
    StepperMotor_Direction_t direction);
void StepperMotor_Stop(void);
void StepperMotor_SetEnabled(bool enabled);
bool StepperMotor_IsBusy(void);
uint32_t StepperMotor_GetRemainingSteps(void);

#endif
