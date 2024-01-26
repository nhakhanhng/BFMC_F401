/*
 * Encoder.c
 *
 *  Created on: Nov 15, 2023
 *      Author: nhakh
 */

#include "../../Hardware/Encoder/Encoder.h"

osThreadAttr_t Encoder_attr = {
		.name = "Encoder",
		.stack_size = 64 * 4,
		.priority = osPriorityHigh,
};

void Encoder_Thread(void *arg)
{
	Encoder_HandlerStruct *Encoder = (Encoder_HandlerStruct *)arg;
	uint64_t startTime = 0;
	int delayTime = 0;
	while (1)
	{
		if (!Encoder->ThrEn)
		{
//			hal_encod
			osThreadSuspend(Encoder->runThread);
//			Encoder_Stop(Encoder);
		}
		startTime = osKernelGetTickCount();
//		HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_SET);
//		HAL_GPIO_TogglePin(LEDG_GPIO_Port, LEDG_Pin);
		Encoder_Handle(Encoder);
		delayTime = Encoder->Interval - (osKernelGetTickCount() - startTime);
		osDelay(delayTime > 0? delayTime : 0);
	}
}

void Encoder_Init(Encoder_HandlerStruct *Encoder,TIM_HandleTypeDef *Enc_htim,uint32_t Channel,uint32_t PulsePerRound,uint32_t PulsePerMeter,uint8_t Interval,float LPF_Alp)
{
	Encoder->Enc_htim = Enc_htim;
//	Encoder->Read_htim = Read_htim;
	Encoder->Channel = Channel;
	Encoder->PulsepPerMeter = PulsePerMeter;
	Encoder->Interval = Interval;
	Encoder->LPF_Alp = LPF_Alp;
	Encoder->PulsePerRound = PulsePerRound;
	Encoder->ThrEn = 0;
	Encoder->runThread = osThreadNew(Encoder_Thread, Encoder, &Encoder_attr);
}

void Encoder_Start(Encoder_HandlerStruct *Encoder)
{
	Encoder->Current_Count = 0;
	Encoder->Current_Speed = 0;
	Encoder->TotalCount = 0;
	HAL_TIM_Encoder_Start(Encoder->Enc_htim, Encoder->Channel);
	Encoder->ThrEn = 1;
	osThreadResume(Encoder->runThread);
//	__HAL_TIM_SET_AUTORELOAD(Encoder->Read_htim,Encoder->Interval*1000 - 1);
//	HAL_TIM_Base_Start_IT(Encoder->Read_htim);
}
void Encoder_Stop(Encoder_HandlerStruct *Encoder)
{
	HAL_TIM_Encoder_Stop(Encoder->Enc_htim, Encoder->Channel);
	Encoder->ThrEn = 0;
	osThreadSuspend(Encoder->runThread);
//	HAL_TIM_Base_Stop(Encoder->Read_htim);
}


void Encoder_Handle(Encoder_HandlerStruct *Encoder)
{
	Encoder->Current_Count = __HAL_TIM_GET_COUNTER(Encoder->Enc_htim);
	Encoder->Dir = 0;
	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(Encoder->Enc_htim))
	{
		Encoder->Current_Count = 65535 - Encoder->Current_Count;
		Encoder->Dir = 1;
	}
	Encoder->Current_Speed = (1 - Encoder->LPF_Alp) * Encoder->Current_Speed + Encoder->LPF_Alp * Encoder->Current_Count / Encoder->Interval;
	__HAL_TIM_SET_COUNTER(Encoder->Enc_htim,0);
	Encoder->TotalCount += Encoder->Current_Count;
}

//uint32_t Encoder_GetCount(Encoder_HandlerStruct *Encoder);
uint32_t Encoder_GetTotalCount(Encoder_HandlerStruct *Encoder)
{
	return Encoder->TotalCount;
}
void Encoder_ClearTotalCount(Encoder_HandlerStruct *Encoder)
{
	Encoder->TotalCount = 0;
}

float Encoder_GetSpeed_MPS(Encoder_HandlerStruct *Encoder)
{
	float Speed_PPS = Encoder->Current_Count / Encoder->Interval * 1000;
	return Speed_PPS / Encoder->PulsepPerMeter;
}

float Encoder_GetSpeed_PPS(Encoder_HandlerStruct *Encoder)
{
	return (float)Encoder->Current_Count / Encoder->Interval * 1000;
//	return Encoder->Current_Speed;
}

float Encoder_GetSpeed_RPS(Encoder_HandlerStruct *Encoder)
{
	return (float)1.0 * Encoder_GetSpeed_PPS(Encoder)/PPR;
}

float Encoder_GetRotationCount(Encoder_HandlerStruct *Encoder)
{
	return Encoder->TotalCount / Encoder->PulsePerRound;
}

uint32_t Encoder_GetCount(Encoder_HandlerStruct *Encoder)
{
	return Encoder->Current_Count;
}
