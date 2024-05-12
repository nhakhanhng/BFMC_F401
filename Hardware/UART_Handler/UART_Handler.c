/*
 * UART_Handler.c
 *
 *  Created on: Jan 13, 2024
 *      Author: nhakh
 */

#include "UART_Handler/UART_Handler.h"
#include <string.h>
#include <stdarg.h>
#include "cmsis_os.h"

void UART_Init(UARTHandler_Struct *UART, UART_HandleTypeDef *huart) {
	UART->huart = huart;
//	UART->RxBufferSize = 30;
	UART->RxBuffer = pvPortMalloc(UART->RxBufferSize);
	UART->RxMutex = osMutexNew(NULL);
	UART->TxMutex = osMutexNew(NULL);
	UART->EventFlags = osEventFlagsNew(NULL);
}
HAL_StatusTypeDef UART_printf(UARTHandler_Struct *UART, char *fmt, ...) {
	va_list arg;
	int status = HAL_OK;
	char Buffer[70] = { 0 };
	size_t size = sizeof(Buffer);
	va_start(arg, fmt);
	char *Output = vasnprintf(Buffer, &size, fmt, arg);
	va_end(arg);
	if (Output != NULL) {
		//		while(HAL_UART_GetState(UART->huart) == HAL_UART_STATE_BUSY_RX);
		osMutexAcquire(UART->TxMutex, 10);
		status = HAL_UART_Transmit(UART->huart, (uint8_t*) Output, size, 10);
		osMutexRelease(UART->TxMutex);

		if (Output != Buffer) {

			vPortFree(Output);
		}
	}
	return 0;
}
HAL_StatusTypeDef UART_Receive_ToIdle(UARTHandler_Struct *UART,
		uint8_t *RcvBuff, uint16_t *RcvLen, uint16_t BuffSize, uint32_t timeout) {
	int Status;
	Status = osMutexAcquire(UART->RxMutex, timeout);
	if (Status == osErrorTimeout)
		return HAL_TIMEOUT;
	else if (Status < 0)
		return HAL_ERROR;

	Status = HAL_UARTEx_ReceiveToIdle_DMA(UART->huart, UART->RxBuffer,
			BuffSize);

	if (Status != HAL_OK) {
		HAL_UART_AbortReceive(UART->huart);
		osMutexRelease(UART->RxMutex);
		return Status;
	}

	Status = osEventFlagsWait(UART->EventFlags, UART_RcvToIdleCpl_Event,
	osFlagsWaitAll, timeout);

	if (Status > 0) {
		*RcvLen = UART->RxLen;
		memcpy(RcvBuff, UART->RxBuffer, UART->RxLen);
		memset(UART->RxBuffer, 0, UART->RxLen);
		UART->RxLen = 0;
	}
	HAL_UART_AbortReceive(UART->huart);
	osEventFlagsClear(UART->EventFlags, UART_RcvToIdleCpl_Event);
	osMutexRelease(UART->RxMutex);
	return HAL_OK;
}
HAL_StatusTypeDef UART_Receive(UARTHandler_Struct *UART, uint8_t *RcvBuff,
		uint16_t *RcvLen, uint16_t BuffSize, uint32_t timeout) {
	int Status;
	Status = osMutexAcquire(UART->RxMutex, timeout);
	if (Status == osErrorTimeout)
		return HAL_TIMEOUT;
	else if (Status < 0)
		return HAL_ERROR;

	Status = HAL_UART_Receive_DMA(UART->huart, UART->RxBuffer, RcvLen);
	if (Status != HAL_OK) {
		osMutexRelease(UART->RxMutex);
		return Status;
	}

	Status = osEventFlagsWait(UART->EventFlags, UART_RcvCpl_Event,
	osFlagsWaitAll, timeout);

	if (Status > 0) {
		memcpy(RcvBuff, UART->RxBuffer, RcvLen);
		memset(UART->RxBuffer, 0, RcvLen);
		UART->RxLen = 0;
	} else {
		HAL_UART_AbortReceive(UART->huart);
		osEventFlagsClear(UART->EventFlags, UART_RcvToIdleCpl_Event);
	}
	osMutexRelease(UART->RxMutex);
	return HAL_OK;
}
void UART_Rcv_CB(UARTHandler_Struct *UART) {
	osEventFlagsSet(UART->EventFlags, UART_RcvCpl_Event);
}
void UART_RcvToIdle_CB(UARTHandler_Struct *UART, uint8_t RcvLen) {
	UART->RxLen = RcvLen;
	//	HAL_UART_Transmit_IT(UART->huart, UART->, Size)
	osEventFlagsSet(UART->EventFlags, UART_RcvToIdleCpl_Event);
}
