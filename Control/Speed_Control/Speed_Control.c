/*
 * Speed_Control.c
 *
 *  Created on: Nov 21, 2023
 *      Author: nhakh
 */

#include "Speed_Control/Speed_Control.h"

static osThreadAttr_t SpeedController_attr = {
		.name = "SpdCtrl",
		.stack_size = 128 * 4,
		.priority = osPriorityHigh,
};

static void SpeedControl_Thread(void *arguments)
{
	SpeedControl_HandleStruct *Controller = (SpeedControl_HandleStruct *)arguments;
	uint32_t startTime = 0;
	int delayTime;
	while (1)
	{
		if (!Controller->EnThr) {
			osThreadSuspend(Controller->runThread);
		}
		startTime = osKernelGetTickCount();
		float duty = PID_calc(Controller->PID, Encoder_GetSpeed_RPS(Controller->Encoder));
		Motor_setPWM(Controller->Motor, duty);
		delayTime = Controller->PID->dt * 1000 - (osKernelGetTickCount() - startTime);
		osDelay(delayTime > 0? delayTime : 0);
	}
}

void SpeedControl_Init(SpeedControl_HandleStruct *Controller,
		Encoder_HandlerStruct *Encoder, Motor_HandlerStruct *Motor,
		PID_HandleStruct *PID) {
	Controller->Encoder = Encoder;
	Controller->Motor = Motor;
	Controller->PID = PID;

	Controller->runThread = osThreadNew(SpeedControl_Thread, Controller, &SpeedController_attr);
//	osThreadSuspend(Controller->runThread);
	Controller->EnThr = 0;
}

void SpeedControl_setInterval(SpeedControl_HandleStruct *Controller,
		float Interval) {
	Controller->PID->dt = Interval;
}

void SpeedControl_Start(SpeedControl_HandleStruct *Controller) {
	Controller->EnThr = 1;
	osThreadResume(Controller->runThread);
}
void SpeedControl_Stop(SpeedControl_HandleStruct *Controller) {
	Controller->EnThr = 0;
	PID_ClearErr(Controller->PID);
	osThreadSuspend(Controller->runThread);
}
void SpeedControl_setSpeed(SpeedControl_HandleStruct *Controller, float speed)
{
	PID_ClearErr(Controller->PID);
	PID_setSetPoint(Controller->PID, speed);
}

float SpeedControl_getSpeed(SpeedControl_HandleStruct *Controller);

void SpeedControl_setTuning(SpeedControl_HandleStruct *Controller, float Kp,
		float Ki, float Kd, float dt) {
	PID_tunning(Controller->PID, Kp, Ki, Kd, dt);
}


