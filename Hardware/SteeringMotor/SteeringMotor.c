/*
 * SteeringMotor.c
 *
 *  Created on: Feb 22, 2024
 *      Author: nhakh
 */

#include "SteeringMotor.h"


static void Steering_ConfigFreq(Steering_HandlerStruct* Steer)
{
	TIM_TypeDef* TIM = Steer->htim->Instance;
	TIM->PSC = 83;
	TIM->ARR = 20000-1;
}

static uint16_t Steering_Ang2PWM(float Angle)
{
	return 1500 + Angle/0.045 * 0.82;
}

void Steering_Start(Steering_HandlerStruct* Steer)
{
	Steering_ConfigFreq(Steer);
	HAL_TIM_PWM_Start(Steer->htim, Steer->Channel);
	Steering_setAngle(Steer,0);
}
void Steering_setAngle(Steering_HandlerStruct* Steer, float Angle)
{
	if (Angle > Steer->posLimit) Angle = Steer->posLimit;
	else if(Angle< Steer->negLimit) Angle = Steer->negLimit;

	Steering_setPWM(Steer, Steering_Ang2PWM(Angle + Steer->Offset ));
}
void Steering_setPWM(Steering_HandlerStruct* Steer, uint32_t PWM)
{
	__HAL_TIM_SET_COMPARE(Steer->htim, Steer->Channel, PWM);
}
uint8_t Steering_inRange(Steering_HandlerStruct* Steer, float Angle)
{
	return Steer->negLimit <= Angle && Angle <= Steer->posLimit;
}

void Steering_SetAngleCMD(void* SteerHandler, char* InMsg, char* outResp)
{
	Steering_HandlerStruct* Steering = (Steering_HandlerStruct*) SteerHandler;
	float Angle =0;
	uint32_t res = sscanf(InMsg, "%f", &Angle);
	if(res != 1)
	{
		sprintf(outResp, "-2;");
		return;
	}

	Steering_setAngle(Steering, Angle);
	sprintf(outResp, "0;");
	return;
}

