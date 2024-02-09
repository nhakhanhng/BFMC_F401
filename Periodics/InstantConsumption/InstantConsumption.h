/*
 * InstantConsumption.h
 *
 *  Created on: Jan 22, 2024
 *      Author: nhakh
 */

#ifndef INSTANTCONSUMPTION_INSTANTCONSUMPTION_H_
#define INSTANTCONSUMPTION_INSTANTCONSUMPTION_H_

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
//	uint8_t isEnable;
	float value;
} ISPublisher_Struct;

void ISPublisher_Start(ISPublisher_Struct *ISPub);
void ISPublisher_EnPubCMD(void *Handler,char *Msg,char *Resp);

#endif /* INSTANTCONSUMPTION_INSTANTCONSUMPTION_H_ */
