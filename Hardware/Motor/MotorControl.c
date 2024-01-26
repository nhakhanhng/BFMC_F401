/*
 * MotorControl.c
 *
 *  Created on: Nov 15, 2023
 *      Author: nhakh
 */

#include "../Motor/MotorControl.h"
#include <stdio.h>

const int speedValuesP[25] = { 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
		18, 19, 20, 21, 22, 26, 30, 35, 40, 45, 50 };
const int speedValuesN[25] = { -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -14,
		-15, -16, -17, -18, -19, -20, -21, -22, -26, -30, -35, -40, -45, -50 };
const int stepValues[25] = { 107, 88, 76, 67, 60, 55, 51, 47, 43, 41, 39, 37,
		35, 034, 33, 32, 30, 29, 28, 25, 24, 021, 19, 18, 17 };

float step_value = 0.00051;
void Motor_Init(Motor_HandlerStruct *Motor, TIM_HandleTypeDef *htim,
		uint32_t channel) {
	Motor->htim = htim;
	Motor->Channel = channel;

}
void Motor_setPWM(Motor_HandlerStruct *Motor, float PWM) {
	uint32_t pulse = round(PWM);
//	if (PWM > 0) {
//	if (PWM < (ZERO * 42000))
//		PWM = floor(PWM);
//	else
//		PWM = round(PWM);
	__HAL_TIM_SET_COMPARE(Motor->htim, Motor->Channel, pulse);
//	} else {
//		__HAL_TIM_SET_COMPARE(Motor->htim, Motor->Forward_channel, 0);
//		__HAL_TIM_SET_COMPARE(Motor->htim, Motor->Backward_channel, Val);
//	}
}

float interpolate(float speed, const int speedsP[], const int speedsN[],
		const int steps[], int size) {
	if (speed <= speedsP[0]) {
		if (speed >= speedsN[0]) {
			return steps[0];
		} else {
			for (int i = 1; i < size; i++) {
				if (speed >= speedsN[i]) {
					float slope = 1.0 * (steps[i] - steps[i - 1])
							/ (speedsN[i] - speedsN[i - 1]);
					return steps[i - 1] + slope * (speed - speedsN[i - 1]);
				}
			}
		}

	}
	if (speed >= speedsP[size - 1])
		return steps[size - 1];
	if (speed <= speedsN[size - 1])
		return steps[size - 1];

	for (int i = 1; i < size; i++) {
		if (speed <= speedsP[i]) {
			float slope = 1.0 * (steps[i] - steps[i - 1])
					/ (speedsP[i] - speedsP[i - 1]);
			return steps[i - 1] + slope * (speed - speedsP[i - 1]);
		}
	}

	return -1;
}

void Motor_setSpeed(Motor_HandlerStruct *Motor, float Speed) {
//	Speed = -Speed;
	step_value = interpolate(-Speed, speedValuesP, speedValuesN, stepValues,
			25);
	float pulse = (((float) step_value * Speed + ZERO) * 60);
	pulse /= 100;
	Motor_setPWM(Motor, pulse);
}

void Motor_Start(Motor_HandlerStruct *Motor) {
	HAL_TIM_PWM_Start(Motor->htim, Motor->Channel);
//	__HAL_TIM_SET_COMPARE(Motor->htim, Motor->Channel, round(ZERO*42)/100);
	Motor_setSpeed(Motor, 0);
}

void Motor_setSpeedCMD(void *Handler, char *InMsg, char *Resp) {
	Motor_HandlerStruct *Motor = (Motor_HandlerStruct*) Handler;
	float Speed = 0;
	uint32_t res = sscanf(InMsg, "%f", &Speed);
	if (res != 1) {
		sprintf(Resp, "-1;;");
		return;
	}
	Motor_setSpeed(Motor, -Speed);
	sprintf(Resp, "0;;");
}
void Motor_BrakeCMD(void *Handler, char *InMsg, char *Resp) {
	return;
}
