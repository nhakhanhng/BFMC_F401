/*
 * Servo.h
 *
 *  Created on: Nov 15, 2023
 *      Author: nhakh
 */

#ifndef SERVO_SERVO_H_
#define SERVO_SERVO_H_
#include "main.h"

#define ZERO		0.07672070

typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t Channel;
	float Limit;
	float Offset;
} Servo_HandleStruct;

typedef struct {
	float step_value;
	float default_value;
} Interpolation_Result;

void Servo_Init(Servo_HandleStruct *Servo,TIM_HandleTypeDef *htim,uint32_t Channel,float Limit,float Offset);

HAL_StatusTypeDef Servo_Start(Servo_HandleStruct *Servo);

void Servo_setPWM(Servo_HandleStruct *Servo,float PWM);
void Servo_setAngle(Servo_HandleStruct* Servo,float Angle);
void Servo_setAngleCMD(void *Handler,char* InMsg,char* Resp);
#endif /* SERVO_SERVO_H_ */
