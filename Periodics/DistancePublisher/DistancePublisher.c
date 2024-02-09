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

	while (1) {
		if (!Publisher->Period) {
			osThreadSuspend(Publisher->PubThread);
		}
		int16_t Range[4] = { 0 };
		for (int i = 0; i < 4; i++) {
//			Range[i] = Vl53l1_ReadMeasure(&Publisher->Vl53l1[i]);
			Range[i] = 10;
		}
		UART_printf(Publisher->UART_Handler, "@%d:%d;%d;%d;%d;;\r\n",
				Publisher->PubID, Range[0], Range[1], Range[2], Range[3]);
		osDelay(Publisher->Period);
	}
}

void DistPublisher_Start(DistPublisher_Struct *Publisher) {
//	for (int i = 0; i < 4; i++) {
//		Vl53l1_Init(&Publisher->Vl53l1[i], Publisher->i2c_dev);
//	}
	const osThreadAttr_t DistPub_TaskAttr = { .name = "Distance Publisher",
			.stack_size = 512 * 4, .priority = osPriorityNormal, };
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
