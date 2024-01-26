/*
 * PID_Control.c
 *
 *  Created on: Nov 15, 2023
 *      Author: nhakh
 */

#include "PID_Control/PID_Control.h"

void PID_tunning(PID_HandleStruct* Controller,float Kp,float Ki,float Kd,float dt)
{
	Controller->Kp = Kp;
	Controller->Ki = Ki;
	Controller->Kd = Kd;
	Controller->dt = dt;
	Controller->Intergration_Err = 0;
	Controller->Prev_Error = 0;
}
float PID_calc(PID_HandleStruct *Controller,float input)
{
	float Error = Controller->SetPoint - input;
	Controller->Intergration_Err += Error * Controller->dt;
	float duty = Controller->Kp * Error + Controller->Ki * Controller->Intergration_Err + Controller->Kd * (Error - Controller->Prev_Error) / Controller->dt;
	Controller->Prev_Error = Error;
	return duty;
}

void PID_ClearErr(PID_HandleStruct *Controller)
{
	Controller->Prev_Error = 0;
	Controller->Intergration_Err = 0;
}
void PID_setSetPoint(PID_HandleStruct *Controller,float SetPoint)
{
	Controller->SetPoint = SetPoint;
	PID_ClearErr(Controller);
}
