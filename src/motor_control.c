#include "motor_control.h"

/*
 * File: motor_control.c
 * Chức năng:
 * - Khởi tạo các chân GPIO điều khiển động cơ.
 * - Điều khiển hướng chạy: tiến, lùi, rẽ trái, rẽ phải, dừng.
 * - Tạo xung đơn giản bằng delay để thay đổi tốc độ.
 */

static void delay_cycles(uint32_t t)
{
	int i,j;
	for(i=0;i<t;i++){
		for(j=0;j<t;j++){
			}
		}
}

void MotorControl_Init(void)
{
    RCC->APB2ENR |= (1 << 2); // IOPAEN

    GPIOA->CRL &= ~(0xF << 0);
    GPIOA->CRL |= (0x2 << 0);  // PA0 output push-pull 2 MHz

    GPIOA->CRL &= ~(0xF << 4);
    GPIOA->CRL |= (0x2 << 4);  // PA1 output push-pull 2 MHz

    GPIOA->CRL &= ~(0xF << 16);
    GPIOA->CRL |= (0x2 << 16); // PA4 output push-pull 2 MHz

    GPIOA->CRL &= ~(0xF << 20);
    GPIOA->CRL |= (0x2 << 20); // PA5 output push-pull 2 MHz

    GPIOA->CRL &= ~(0xF << 24);
    GPIOA->CRL |= (0x2 << 24); // PA6 output push-pull 2 MHz

    GPIOA->CRL &= ~(0xF << 28);
    GPIOA->CRL |= (0x2 << 28); // PA7 output push-pull 2 MHz
}

void MotorControl_SetSpeedPulse(uint16_t pulse_delay)
{
    GPIOA->ODR |= ((1 << 0) | (1 << 1));
    delay_cycles(pulse_delay);
    GPIOA->ODR &= ~((1 << 0) | (1 << 1));
    delay_cycles(1000);
}

void Motor_MoveForward(void)
{
    GPIOA->ODR |= (1 << 4) | (1 << 6);
    GPIOA->ODR &= ~((1 << 5) | (1 << 7));
}

void Motor_MoveBackward(void)
{
    GPIOA->ODR |= (1 << 5) | (1 << 7);
    GPIOA->ODR &= ~((1 << 4) | (1 << 6));
}

void Motor_TurnLeft(void)
{
    GPIOA->ODR |= (1 << 4) | (1 << 7);
    GPIOA->ODR &= ~((1 << 5) | (1 << 6));
}

void Motor_TurnRight(void)
{
    GPIOA->ODR |= (1 << 6) | (1 << 5) ;
    GPIOA->ODR &= ~((1 << 4) | (1 << 7));
}

void Motor_Stop(void)
{
    GPIOA->ODR &= ~((1 << 4) | (1 << 5) | (1 << 6) | (1 << 7));
}
