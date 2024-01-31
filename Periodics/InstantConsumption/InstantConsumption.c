/*
 * InstantConsumption.c
 *
 *  Created on: Jan 22, 2024
 *      Author: nhakh
 */

#include "InstantConsumption.h"

void ISPublisher_Thread(void *arg) {
	ISPublisher_Struct *ISPub = (ISPublisher_Struct*) arg;

	while (1) {
		if (!ISPub->isEnable)
			osThreadSuspend(ISPub->PubThread);
		uint32_t raw = ADC_GetValueFromeChannel(ISPub->ADC_Handler,
				ISPub->ADC_Channel);
		ISPub->value = 1.0 * raw / 19859.39;
		UART_printf(ISPub->UART_Handler, "@6:%.3f;;\r\n", ISPub->value);
		osDelay(ISPub->Period);
	}
}

void ISPublisher_Start(ISPublisher_Struct *ISPub) {
	const osThreadAttr_t ISPub_TaskAttr = { .name =
			"Instant Consumption Publisher", .stack_size = 512 * 4, .priority =
			osPriorityNormal, };
	ISPub->PubThread = osThreadNew(ISPublisher_Thread, ISPub, &ISPub_TaskAttr);
}

void ISPublisher_EnPubCMD(void *Handler, char *Msg, char *Resp) {
	ISPublisher_Struct *Publisher = (ISPublisher_Struct*) Handler;
	int enable = 0;
	uint32_t res = sscanf(Msg, "%d", &enable);
	if (res != 1) {
		sprintf(Resp, "-1;");
		return;
	}
	Publisher->isEnable = enable;
	if (enable) osThreadResume(Publisher->PubThread);
	sprintf(Resp, "0;");
	return ;
}
