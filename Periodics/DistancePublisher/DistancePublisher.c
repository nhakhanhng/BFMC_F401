/*
 * DistancePublisher.c
 *
 *  Created on: Feb 3, 2024
 *      Author: nhakh
 */

#include "DistancePublisher.h"

void DistPublisher_Thread(void *arg) {
	DistPublisher_Struct *Publisher = (DistPublisher_Struct*) arg;
//	uint32_t startTime = 0;

	int16_t Range[4] = { 0 };
	while (1) {
		if (!Publisher->Period) {
			osThreadSuspend(Publisher->PubThread);
		}
		for (int i = 0; i < Publisher->NumofSens; i++) {
			Range[i] = Vl53l0x_ReadMeasure(&Publisher->Vl53l0[i]);
//			Range[i] = 10;
		}
		UART_printf(Publisher->UART_Handler, "@%d:%d;%d;%d;%d;;\r\n",
				Publisher->PubID, Range[0], Range[1], Range[2],Range[3]);
		osDelay(Publisher->Period);
	}
}

void DistPublisher_Start(DistPublisher_Struct *Publisher) {
	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	for (int i = 0; i < Publisher->NumofSens; i++) {
		status = Vl53l0x_Init(&Publisher->Vl53l0[i], Publisher->i2c_dev);
		if (status != VL53L0X_ERROR_NONE) {
			return;
//			Error_Handler();
		}
	}
	const osThreadAttr_t DistPub_TaskAttr = { .name = "Distance Publisher",
			.stack_size = 512 * 4, .priority = osPriorityBelowNormal, };
	Publisher->Period = 0;
	Publisher->PubID = 8;
	Publisher->PubThread = osThreadNew(DistPublisher_Thread, Publisher,
			&DistPub_TaskAttr);
}
void DistPublisher_EnPubCMD(void *Handler, char *Msg, char *Resp) {
	DistPublisher_Struct *Publisher = (DistPublisher_Struct*) Handler;
	int period = 0;
	uint32_t res = sscanf(Msg, "%d", &period);
	if (res != 1) {
		sprintf(Resp, "-1;");
		return;
	}
	Publisher->Period = period;
	if (period)
		osThreadResume(Publisher->PubThread);
	sprintf(Resp, "0;");
	return;
}
