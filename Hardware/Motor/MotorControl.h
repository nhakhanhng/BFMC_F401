#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_
#include "main.h"

#include "../../Hardware/GPIO/GPIOHandler.h"
//#include "stdio.h"

#define ZERO			7456.8

typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t Channel;

	int8_t Limit;
	int8_t Zero;
} Motor_HandlerStruct;

void Motor_Init(Motor_HandlerStruct *Motor, TIM_HandleTypeDef *htim, uint32_t channel);
void Motor_setPWM(Motor_HandlerStruct *Motor,float PWM);
void Motor_setDutyCycle(Motor_HandlerStruct *Motor,float value);
void Motor_setSpeed(Motor_HandlerStruct *Motor,float Speed);
void Motor_Start(Motor_HandlerStruct *Motor);
void Motor_setSpeedCMD(void *Handler,char* InMsg,char* Resp);
void Motor_BrakeCMD(void* Handler, char* InMsg, char* Resp);
#endif
