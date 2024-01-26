/*
 * SerialMonitor.h
 *
 *  Created on: Jan 13, 2024
 *      Author: nhakh
 */

#ifndef UART_HANDLER_H_
#define UART_HANDLER_H_

#define UART_RcvCpl_Event 		0b0001
#define UART_RcvToIdleCpl_Event 0b0001

#include "main.h"
#include "cmsis_os.h"

typedef struct UARTHandler_Struct
{
	UART_HandleTypeDef *huart;
	osMutexId_t TxMutex,RxMutex;

	uint8_t *RxBuffer;
	uint16_t RxBufferSize;
	uint16_t RxLen;
	osEventFlagsId_t EventFlags;
} UARTHandler_Struct;

void UART_Init(UARTHandler_Struct *UART,UART_HandleTypeDef *huart);
HAL_StatusTypeDef UART_printf(UARTHandler_Struct *UART,char *fmt,...);
HAL_StatusTypeDef UART_Receive_ToIdle(UARTHandler_Struct *UART,uint8_t *RcvBuff,uint16_t *RcvLen,uint16_t BuffSize,uint32_t timeout);
HAL_StatusTypeDef UART_Receive(UARTHandler_Struct *UART,uint8_t *RcvBuff,uint16_t *RcvLen,uint16_t BuffSize,uint32_t timeout);
void UART_Rcv_CB(UARTHandler_Struct* UART);
void UART_RcvToIdle_CB(UARTHandler_Struct* UART, uint8_t RcvLen);

#endif /* UART_HANDLER_SERIALMONITOR_H_ */
