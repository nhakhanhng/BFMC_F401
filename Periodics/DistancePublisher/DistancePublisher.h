/*
 * DistancePublisher.h
 *
 *  Created on: Feb 3, 2024
 *      Author: nhakh
 */

#ifndef DISTANCEPUBLISHER_DISTANCEPUBLISHER_H_
#define DISTANCEPUBLISHER_DISTANCEPUBLISHER_H_

#include "VL53_Handler/VL53_Handler.h"
#include "UART_Handler/UART_Handler.h"
#include "I2C_Handler/I2C_Handler.h"

typedef struct {
	I2C_OS_HandlerStruct *i2c_dev;
	uint32_t Period;
	osThreadId_t PubThread;
	uint8_t PubID;
//	uint8_t isEnable;
	UARTHandler_Struct *UART_Handler;
	uint8_t NumofSens;
	Vl53l0x_HandlerStruct *Vl53l0;
} DistPublisher_Struct;


void DistPublisher_Start(DistPublisher_Struct *Publisher);
void DistPublisher_EnPubCMD(void *Handler,char *Msg,char *Resp);
#endif /* DISTANCEPUBLISHER_DISTANCEPUBLISHER_H_ */
