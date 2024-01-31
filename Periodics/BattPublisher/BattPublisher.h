/*
 * BattPublisher.h
 *
 *  Created on: Jan 23, 2024
 *      Author: nhakh
 */

#ifndef BATTPUBLISHER_H_
#define BATTPUBLISHER_H_
#include "UART_Handler/UART_Handler.h"
#include "main.h"
#include "ADC/ADCHandler.h"

typedef struct {
	ADC_HandleStruct *ADC_Handler;
	uint32_t ADC_Channel;
	uint32_t Period;
	osThreadId_t PubThread;
	uint8_t PubID;
	UARTHandler_Struct *UART_Handler;
	uint8_t isEnable;
	float value;
} BattPublisher_Struct;

void BattPublisher_Start(BattPublisher_Struct *BattPub);
void BattPublisher_EnPubCMD(void *Handler,char *Msg,char *Resp);

#endif /* BATTPUBLISHER_H_ */
