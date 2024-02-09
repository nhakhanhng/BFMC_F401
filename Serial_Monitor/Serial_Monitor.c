/*
 * Serial_Monitor.c
 *
 *  Created on: Jan 13, 2024
 *      Author: nhakh
 */

#include "Serial_Monitor.h"
#include "string.h"
uint8_t SerMonRcv_Buffer;
extern UARTHandler_Struct UART1;

static inline int SerialMonitor_ParseData(char* PareBuffer, int* MsgID, char* Msg)
{
	// char Data[256];
	char* token = strtok(PareBuffer, ":");       // Get Message ID
	if(token == NULL)
		return -2;
	char strMsgID[3] = {0};
	strcpy(strMsgID, token+1);       //Get ID
	sscanf(strMsgID, "%d", MsgID);

	token = strtok(NULL,":");       //Get Message
	if(token == NULL)
		return -1;
	strcpy(Msg, token);
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	return 0;

}

void SerialMonitor_TaskHandler(void* argument)
{
	SerialMonitor_Struct* SerMon = (SerialMonitor_Struct*) argument;
	int Status;
//	UART_printf(SerMon->UART, "Hello\r\n");
	while(1)
	{
		uint8_t RcvData[SER_MON_RCVBUFFER_SIZE] = {0};
		uint16_t RcvLen = 0;
		Status = UART_Receive_ToIdle(SerMon->UART, RcvData, &RcvLen, SER_MON_RCVBUFFER_SIZE, osWaitForever);
//		SyncPrintf("Rcv Status %d ",Status);
		if (Status == HAL_OK)
		{
//			SyncPrintf("Receive Len %d Data %s",(int) RcvLen, RcvData);
//			UART_printf(SerMon->UART, "%s",RcvData);
			if(RcvData[0] == '#' && RcvData[RcvLen -2] == '\r' && RcvData[RcvLen - 1] == '\n')
			{
				int MsgID = 1;
				char Msg[SER_MON_RCVBUFFER_SIZE];
				int Status = SerialMonitor_ParseData((char*)RcvData, &MsgID, Msg);
//				SyncPrintf("Res %d ID %d Msg %s \r\n", Status, MsgID, Msg);
				if(Status == 0)
				{
					if(1<= MsgID && MsgID <= SerMon->CallbackSize)
					{
						char resp[256] = "0;";
						void (*CB_Handler)(void*, char*, char*) = SerMon->Callback[MsgID-1].Function;
						if(CB_Handler == NULL)
						{
							UART_printf(SerMon->UART, "@%d:-3;;\r\n", MsgID);
						}
						else
						{
							CB_Handler(SerMon->Callback[MsgID-1].ObjHandler, Msg, resp);
							UART_printf(SerMon->UART, "@%d:%s;\r\n", MsgID, resp);
						}

					}
					else
					{
						UART_printf(SerMon->UART, "@%d:-3;;\r\n",MsgID);
					}
				} else if(Status == -1)
				{
					UART_printf(SerMon->UART, "@%d:-1;;\r\n", MsgID);
				}
			}
		}
		else
		{
//			UART_printf(SerMon->UART, "Rcv Timeout \r\n");
		}
	}
}


void SerialMonitor_Start(SerialMonitor_Struct* SerMon)
{
	const osThreadAttr_t SerMon_TaskAtt = {
			.priority = (osPriority_t)osPriorityRealtime,
			.name = "SerialMonitor_Task",
			.stack_size = 1024 * 4
	};
	SerMon->SerMonRcvTask = osThreadNew(SerialMonitor_TaskHandler, (void*)SerMon, &SerMon_TaskAtt);

}


