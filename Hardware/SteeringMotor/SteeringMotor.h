/*
 * SteeringMotor.h
 *
 *  Created on: Feb 22, 2024
 *      Author: nhakh
 */

#ifndef STEERINGMOTOR_STEERINGMOTOR_H_
#define STEERINGMOTOR_STEERINGMOTOR_H_

#include "main.h"
typedef struct Steering_HandlerStruct
{
	TIM_HandleTypeDef* htim;
	uint32_t Channel;
	float negLimit, posLimit;

	float Offset;

} Steering_HandlerStruct;

void Steering_Start(Steering_HandlerStruct* Steer);
void Steering_setAngle(Steering_HandlerStruct* Steer, float Angle);
void Steering_setPWM(Steering_HandlerStruct* Steer, uint32_t PWM);
uint8_t Steering_inRange(Steering_HandlerStruct* Steer, float Angle);

void Steering_SetAngleCMD(void* SteerHandler, char* InStr, char* OutStr);

#endif /* STEERINGMOTOR_STEERINGMOTOR_H_ */
