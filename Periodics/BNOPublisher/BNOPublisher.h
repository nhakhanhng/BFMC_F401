/*
 * BONPublisher.h
 *
 *  Created on: Jan 30, 2024
 *      Author: nhakh
 */

#ifndef BNOPUBLISHER_BNOPUBLISHER_H_
#define BNOPUBLISHER_BNOPUBLISHER_H_

#include "main.h"
#include "BNO055/bno_config.h"
#include "BNO055/bno055.h"
#include "UART_Handler/UART_Handler.h"
#include "cmsis_os.h"

typedef struct {
	I2C_OS_HandlerStruct *BNO_hi2c;
	UARTHandler_Struct *UART_Handler;
	uint32_t Period;
	osThreadId_t PubThread;
//	TIM_HandleTypeDef *htim;
	uint8_t PubID;
} BNOPublisher_Struct;


void BNOPublisher_Start(BNOPublisher_Struct *Publisher);
void BNOPublisher_Run(BNOPublisher_Struct *Publisher);
void BNOPublisher_EnPubCMD(void *Handler,char *Msg,char *Resp);

#endif /* BNOPUBLISHER_BNOPUBLISHER_H_ */
