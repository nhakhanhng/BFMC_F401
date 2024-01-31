/*
 * BNOPublisher.c
 *
 *  Created on: Jan 31, 2024
 *      Author: nhakh
 */
#include "BNOPublisher.h"

void Publisher_Thread(void *arg) {
	BNOPublisher_Struct *Publisher = (BNOPublisher_Struct*) arg;
	uint32_t startTime = 0;
	uint32_t delayTime = 0;
	bno055_vector_t e = bno055_getVectorEuler();
	bno055_vector_t a = bno055_getVectorLinearAccel();
	while (1) {
		startTime = osKernelGetTickCount();
		if (Publisher->isEnable == 0) {
			osThreadSuspend(Publisher->PubThread);
			continue;
		}
		e = bno055_getVectorEuler();
		a = bno055_getVectorLinearAccel();
		UART_printf(Publisher->UART_Handler,
				"@7:%.2f;%.2f%.2f%.2f%.2f%.2f;;\r\n", e.x, e.y, e.z, a.x, a.y,
				a.z);
		delayTime = Publisher->Period - (osKernelGetTickCount() - startTime);
		osDelay((delayTime < 0) ? 0 : delayTime);
	}
}

void BNOPublisher_Start(BNOPublisher_Struct *Publisher) {
	bno055_assignI2C(Publisher->BNO_hi2c);
	bno055_setup();
	bno055_setOperationModeNDOF();
	const osThreadAttr_t BNOPub_TaskAttr = { .name = "BNO Publisher",
			.stack_size = 512 * 4, .priority = osPriorityNormal, };
	Publisher->isEnable = 0;
	Publisher->PubThread = osThreadNew(Publisher_Thread, Publisher,
			&BNOPub_TaskAttr);
}
void BNOPublisher_EnPubCMD(void *Handler, char *Msg, char *Resp) {
	BNOPublisher_Struct *Publisher = (BNOPublisher_Struct*) Handler;
	int enable = 0;
	uint32_t res = sscanf(Msg, "%d", &enable);
	if (res != 1) {
		sprintf(Resp, "-1;");
		return;
	}
	Publisher->isEnable = enable;
	if (enable)
		osThreadResume(Publisher->PubThread);
	sprintf(Resp, "0;");
	return;
}
