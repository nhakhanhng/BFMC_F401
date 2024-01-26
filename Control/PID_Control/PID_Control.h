/*
 * PID_Control.h
 *
 *  Created on: Nov 15, 2023
 *      Author: nhakh
 */

#ifndef PID_CONTROL_PID_CONTROL_H_
#define PID_CONTROL_PID_CONTROL_H_
#include "stdio.h"

typedef struct {
	float Kp,Ki,Kd;
	float SetPoint;
	float dt;
	float Prev_Error;
	float Intergration_Err;
} PID_HandleStruct;

void PID_tunning(PID_HandleStruct* Controller,float Kp,float Ki,float Kd,float dt);
float PID_calc(PID_HandleStruct *Controller,float input);
void PID_ClearErr(PID_HandleStruct *Controller);
void PID_setSetPoint(PID_HandleStruct *Controller,float SetPoint);

#endif /* PID_CONTROL_PID_CONTROL_H_ */
