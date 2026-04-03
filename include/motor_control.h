#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

/* Header khai báo các hàm điều khiển hướng và tốc độ động cơ */

#include "stm32f10x.h"

void MotorControl_Init(void);
void MotorControl_SetSpeedPulse(uint16_t pulse_delay);
void Motor_MoveForward(void);
void Motor_MoveBackward(void);
void Motor_TurnLeft(void);
void Motor_TurnRight(void);
void Motor_Stop(void);

#endif
