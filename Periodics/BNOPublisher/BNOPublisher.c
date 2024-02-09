/*
 * BNOPublisher.c
 *
 *  Created on: Jan 31, 2024
 *      Author: nhakh
 */
#include "BNOPublisher.h"

void BNOPublisher_Thread(void *arg) {
	BNOPublisher_Struct *Publisher = (BNOPublisher_Struct*) arg;
	uint32_t startTime = 0;
	uint32_t delayTime = 0;
	while (1) {
		startTime = osKernelGetTickCount();
		BNOPublisher_Run(Publisher);
		delayTime = Publisher->Period - (osKernelGetTickCount() - startTime);
		osDelay((delayTime < 0) ? 0 : delayTime);
	}
}

void BNOPublisher_Run(BNOPublisher_Struct *Publisher) {
	bno055_vector_t e = {
			.x = 1,.y = 1,.z = 1
	};
	bno055_vector_t a = {
			.x = 1,.y = 1,.z = 1
	};
//	bno055_vector_t e = bno055_getVectorEuler();
//	bno055_vector_t a = bno055_getVectorLinearAccel();
	UART_printf(Publisher->UART_Handler, "@7:%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;;\r\n",
			e.x, e.y, e.z, a.x, a.y, a.z);
}

void BNOPublisher_Start(BNOPublisher_Struct *Publisher) {
//	bno055_assignI2C(Publisher->BNO_hi2c);
//	bno055_setup();
//	bno055_setOperationModeNDOF();
//	const osThreadAttr_t BNOPub_TaskAttr = { .name = "BNO Publisher",
//			.stack_size = 512 * 4, .priority = osPriorityAboveNormal7, };
	Publisher->Period = 0;
//	Publisher->PubThread = osThreadNew(BNOPublisher_Thread, Publisher,
//			&BNOPub_TaskAttr);

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
		__HAL_TIM_SET_AUTORELOAD(Publisher->htim, Publisher->Period * 10);
		HAL_TIM_Base_Start_IT(Publisher->htim);
	}
	else {
		HAL_TIM_Base_Stop_IT(Publisher->htim);
	}
	sprintf(Resp, "0;");
	return;
}
