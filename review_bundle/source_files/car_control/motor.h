#ifndef MOTOR_H_
#define MOTOR_H_

/*
 * speed取值范围：
 *  100.0f：最大正转
 *   30.0f：30%正转
 *    0.0f：停止
 *  -30.0f：30%反转
 * -100.0f：最大反转
 */

void Motor_Init(void);

void Motor_SetLeft(float speed);
void Motor_SetRight(float speed);

void Motor_SetBoth(float left_speed, float right_speed);

void Motor_Stop(void);

#endif /* MOTOR_H_ */