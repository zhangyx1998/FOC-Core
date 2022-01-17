#ifndef __INTERRUPTS_H
#define __INTERRUPTS_H
#include "main.h"
#include "sin.h"
#include <stdint.h>

struct Motor {
	uint32_t* A;
	uint32_t* B;
	uint32_t* C;
} bldc_motor = {&(TIM3->CCR1), &(TIM3->CCR2), &(TIM3->CCR4)};

void driveMotor(struct Motor motor, uint8_t rotation) {
	// Motor Phase Transfer
	*(motor.A) = u8_sin(rotation + 0);
	*(motor.B) = u8_sin(rotation + 85);
	*(motor.C) = u8_sin(rotation + 171);
}
void idleMotor(struct Motor motor) {
	// Motor Phase Transfer
	*(motor.A) = 0;
	*(motor.B) = 0;
	*(motor.C) = 0;
}
// Motor speed configuration
#define PRESCALER_MASK 0b1000
uint8_t rotation = 0, prescaler = 0; // 0 ~ 255
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    driveMotor(
        bldc_motor,
        rotation += 1 //(prescaler & PRESCALER_MASK) && (prescaler ^ (++prescaler) & PRESCALER_MASK)
    );
}

#endif