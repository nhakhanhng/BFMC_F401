/*
 * Speed_Control.h
 *
 *  Created on: Nov 15, 2023
 *      Author: nhakh
 */

#ifndef SPEED_CONTROL_SPEED_CONTROL_H_
#define SPEED_CONTROL_SPEED_CONTROL_H_
#include "cmsis_os.h"
#include "main.h"
#include "Motor/MotorControl.h"
#include "Encoder/Encoder.h"
#include "PID_Control/PID_Control.h"

typedef struct {
	Encoder_HandlerStruct *Encoder;
	Motor_HandlerStruct *Motor;
	PID_HandleStruct *PID;
	osThreadId_t runThread;
	uint8_t EnThr;

} SpeedControl_HandleStruct;

void SpeedControl_Init(SpeedControl_HandleStruct *Controller,Encoder_HandlerStruct *Encoder, Motor_HandlerStruct *Motor,PID_HandleStruct *PID);


void SpeedControl_Start(SpeedControl_HandleStruct *Controller);
void SpeedControl_Stop(SpeedControl_HandleStruct *Controller);
void SpeedControl_setInterval(SpeedControl_HandleStruct *Controller,float Interval);
void SpeedControl_setSpeed(SpeedControl_HandleStruct *Controller,float speed);

float SpeedControl_getSpeed(SpeedControl_HandleStruct *Controller);
void SpeedControl_setTuning(SpeedControl_HandleStruct* Controller,float Kp, float Ki, float Kd,float dt);



#endif /* SPEED_CONTROL_SPEED_CONTROL_H_ */
