#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

void motor_init(void);
void motor_apply(int16_t left_pwm, int16_t right_pwm);
void motor_stop(void);
void motor_standby_enable(int enable);

#endif
