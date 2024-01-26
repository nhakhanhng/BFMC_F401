/*
 * Servo.c
 *
 *  Created on: Nov 15, 2023
 *      Author: nhakh
 */
#include "Servo/Servo.h"
#include <math.h>

static const float steeringValueP[2] = { 15.0, 20.0 };
static const float steeringValueN[2] = { -15.0, -20.0 };
static const float stepValues[2] = { 0.0008594, 0.000951570 };
static const float zeroDefaultValues[2] = { 0.07714891, 0.07672070 };

static Interpolation_Result interpolate(float steering, const float steeringP[],
		const float steeringN[], const float steps[],
		const float zeroDefault[], int size) {
	Interpolation_Result Result;
	// If steering is within the bounds of the first positive and negative reference values
	if (steering <= steeringP[0]) {
		if (steering >= steeringN[0]) {
			Result.step_value = steps[0];
			Result.default_value = zeroDefault[0];
			return Result;
		} else {
			for (int i = 1; i < size; i++) {
				// Find the interval of negative reference values where steering falls into
				if (steering >= steeringN[i]) {
					// Calculate slopes for interpolation
					float slopeStepValue = (steps[i] - steps[i - 1])
							/ (steeringN[i] - steeringN[i - 1]);
					float slopeZeroDefault = (zeroDefault[i]
							- zeroDefault[i - 1])
							/ (steeringN[i] - steeringN[i - 1]);

					// Return the interpolated values
					Result.step_value = steps[i - 1]
							+ slopeStepValue
									* (steering - steeringN[i - 1]);
					Result.default_value = zeroDefault[i - 1]
							+ slopeZeroDefault
									* (steering - steeringN[i - 1]);
					return Result;
				}
			}
		}

	}

	// Boundary conditions for positive and negative reference values
	if (steering >= steeringP[size - 1]
			|| steering <= steeringN[size - 1]) {
		Result.step_value = steps[size - 1];
		Result.default_value = zeroDefault[size - 1];
	}

	// Interpolation for values between positive reference values
	for (int i = 1; i < size; i++) {
		if (steering <= steeringP[i]) {
			// Calculate slopes for interpolation
			float slopeStepValue = (steps[i] - steps[i - 1])
					/ (steeringP[i] - steeringP[i - 1]);
			float slopeZeroDefault = (zeroDefault[i]
					- zeroDefault[i - 1])
					/ (steeringP[i] - steeringP[i - 1]);

			// Return the interpolated values
			Result.step_value = steps[i - 1]
					+ slopeStepValue * (steering - steeringP[i - 1]);
			Result.default_value = zeroDefault[i - 1]
					+ slopeZeroDefault * (steering - steeringP[i - 1]);
			return Result;
		}
	}

	// Default return if no interval is found
	Result.step_value = -1;
	Result.default_value = - 1;
	return Result;
}
;

void Servo_Init(Servo_HandleStruct *Servo, TIM_HandleTypeDef *htim,
		uint32_t Channel, float Limit, float Offset) {
	Servo->htim = htim;
	Servo->Channel = Channel;
	Servo->Limit = Limit;
	Servo->Offset = Offset;
}

HAL_StatusTypeDef Servo_Start(Servo_HandleStruct *Servo) {
//	__HAL_TIM_SET_COMPARE(Servo->htim, Servo->Channel, ZERO);
	HAL_TIM_PWM_Start(Servo->htim, Servo->Channel);
	Servo_setAngle(Servo, 0);
}

//static float Angle2PWM(float Angle) {
//	return 1500 + Angle / 0.045;
//}

void Servo_setPWM(Servo_HandleStruct *Servo, float PWM) {
	__HAL_TIM_SET_COMPARE(Servo->htim, Servo->Channel, round(PWM));
}
void Servo_setAngle(Servo_HandleStruct *Servo, float Angle) {
//	if (Angle > Servo->Limit)
//		Angle = Servo->Limit;
//	else if (Angle < -Servo->Limit)
//		Angle = -Servo->Limit;
	Interpolation_Result interpolation = interpolate(Angle,steeringValueP,steeringValueN,stepValues,zeroDefaultValues,2);
	float pulse = (interpolation.step_value * Angle + interpolation.default_value) * 60000;
	Servo_setPWM(Servo, pulse);
}

void Servo_setAngleCMD(void *Handler, char *InMsg, char *Resp) {
	Servo_HandleStruct *Servo = (Servo_HandleStruct*) Handler;
	float Angle = 0;
	uint32_t res = sscanf(InMsg, "%f", &Angle);
	if (res != 1) {
		sprintf(Resp, "-1;");
		return;
	}
	Servo_setAngle(Servo, Angle);
	sprintf(Resp, "0;");
}
