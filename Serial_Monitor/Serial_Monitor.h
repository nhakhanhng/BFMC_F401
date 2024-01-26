/*
 * Serial_Monitor.h
 *
 *  Created on: Jan 13, 2024
 *      Author: nhakh
 */

#ifndef SERIAL_MONITOR_H_
#define SERIAL_MONITOR_H_

#include "main.h"
#include "UART_Handler/UART_Handler.h"
#include "cmsis_os.h"

#define SER_MON_RCVBUFFER_SIZE 50

#define SER_MON_RCVDONE_FLAG 0b0001

typedef struct SerialMonitor_CBStruct
{
	void (*Function) (void*, char*, char*);
	void* ObjHandler;
} SerialMonitor_CBStruct;

typedef struct SerialMonitor_Struct
{
	SerialMonitor_CBStruct* Callback;
	uint8_t CallbackSize;
	UARTHandler_Struct* UART;

	osThreadId_t SerMonRcvTask;
} SerialMonitor_Struct;



void SerialMonitor_Start(SerialMonitor_Struct* SerMon);
void SerialMonitor_CBHandler(SerialMonitor_Struct* SerMon, uint8_t RcvLen);

#endif /* SERIAL_MONITOR_H_ */
