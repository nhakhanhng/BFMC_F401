/*
 * Encoder.h
 *
 *  Created on: Nov 15, 2023
 *      Author: nhakh
 */

#ifndef ENCODER_ENCODER_H_
#define ENCODER_ENCODER_H_
#include "main.h"
#include "cmsis_os.h"

#define PPR			660

typedef struct {
	TIM_HandleTypeDef *Enc_htim;
//	TIM_HandleTypeDef *Read_htim;
	uint32_t Channel;
	uint32_t Current_Count;
	float Current_Speed;
	uint32_t TotalCount;
	uint32_t PulsepPerMeter;
	uint32_t PulsePerRound;
	float LPF_Alp;
	uint8_t Interval; //ms
	osThreadId_t runThread;
	uint8_t ThrEn;
	uint8_t Dir;
} Encoder_HandlerStruct;

void Encoder_Init(Encoder_HandlerStruct *Encoder,TIM_HandleTypeDef *Enc_htim,uint32_t Channel,uint32_t PulsePerRound,uint32_t PulsePerMeter,uint8_t Interval,float LPF_Alp);
void Encoder_Start(Encoder_HandlerStruct *Encoder);
void Encoder_Stop(Encoder_HandlerStruct *Encoder);

void Encoder_Handle(Encoder_HandlerStruct *Encoder);

uint32_t Encoder_GetCount(Encoder_HandlerStruct *Encoder);
uint32_t Encoder_GetTotalCount(Encoder_HandlerStruct *Encoder);
void Encoder_ClearTotalCount(Encoder_HandlerStruct *Encoder);

float Encoder_GetSpeed_MPS(Encoder_HandlerStruct *Encoder);
float Encoder_GetSpeed_PPS(Encoder_HandlerStruct *Encoder);
float Encoder_GetSpeed_RPS(Encoder_HandlerStruct *Encoder);

float Encoder_GetRotationCount(Encoder_HandlerStruct *Encoder);


#endif /* ENCODER_ENCODER_H_ */
