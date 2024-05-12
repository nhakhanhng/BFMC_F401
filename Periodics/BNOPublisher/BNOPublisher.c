/*
 * BNOPublisher.c
 *
 *  Created on: Jan 31, 2024
 *      Author: nhakh
 */
#include "BNOPublisher.h"

void BNOPublisher_Thread(void *arg) {
	BNOPublisher_Struct *Publisher = (BNOPublisher_Struct*) arg;
	osDelay(10);
	uint32_t startTime = 0;
	uint32_t delayTime = 0;
	while (1) {
		if (Publisher->Period == 0) {
			osThreadSuspend(Publisher->PubThread);
		}
		startTime = osKernelGetTickCount();
		BNOPublisher_Run(Publisher);
		delayTime = Publisher->Period - (osKernelGetTickCount() - startTime);
		osDelay((delayTime < 0) ? 0 : delayTime);
	}
}

void BNOPublisher_Run(BNOPublisher_Struct *Publisher) {
//	bno055_vector_t e = {
//			.x = 1,.y = 1,.z = 1
//	};
//	bno055_vector_t a = {
//			.x = 1,.y = 1,.z = 1
//	};
	bno055_vector_t e = bno055_getVectorEuler();
	bno055_vector_t a = bno055_getVectorLinearAccel();
	UART_printf(Publisher->UART_Handler,
			"@7:%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;;\r\n", e.x, e.y, e.z, a.x, a.y,
			a.z);
}

void BNOPublisher_Start(BNOPublisher_Struct *Publisher) {

	const osThreadAttr_t BNOPub_TaskAttr = { .name = "BNO Publisher",
			.stack_size = 1024 * 4, .priority = osPriorityNormal7, };
	Publisher->Period = 0;
	bno055_assignI2C(Publisher->BNO_hi2c);
	bno055_setAddress(BNO055_I2C_ADDR_LO);
	bno055_setup();
	Publisher->PubThread = osThreadNew(BNOPublisher_Thread, (void*) Publisher,
			&BNOPub_TaskAttr);
//	osThreadSuspend(Publisher->PubThread);
}
void BNOPublisher_EnPubCMD(void *Handler, char *Msg, char *Resp) {
	BNOPublisher_Struct *Publisher = (BNOPublisher_Struct*) Handler;
	int period = 0;
	uint32_t res = sscanf(Msg, "%d", &period);
	if (res != 1) {
		sprintf(Resp, "-1;");
		return;
	}
	Publisher->Period = period;
	if (period > 0) {
//		__HAL_TIM_SET_AUTORELOAD(Publisher->htim, Publisher->Period * 10);
//		HAL_TIM_Base_Start_IT(Publisher->htim);
		Publisher->Period = period;

		bno055_setOperationModeNDOF();
		osThreadResume(Publisher->PubThread);
	} else {
//		HAL_TIM_Base_Stop_IT(Publisher->htim);
		osThreadSuspend(Publisher->PubThread);
	}
	sprintf(Resp, "0;");
	return;
}
