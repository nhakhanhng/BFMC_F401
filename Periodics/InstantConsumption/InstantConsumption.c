/*
 * InstantConsumption.c
 *
 *  Created on: Jan 22, 2024
 *      Author: nhakh
 */

#include "InstantConsumption.h"

void ISPublisher_Thread(void *arg) {
	ISPublisher_Struct *ISPub = (ISPublisher_Struct*) arg;
//	float value = 0;
	while (1) {
		if (!ISPub->Period)
			osThreadSuspend(ISPub->PubThread);
		uint32_t raw = ADC_GetValueFromeChannel(ISPub->ADC_Handler,
				ISPub->ADC_Channel);
		ISPub->value = 1.0 * raw / 19859.39;
//		value = 10;
		UART_printf(ISPub->UART_Handler, "@6:%.3f;;\r\n", ISPub->value);
		osDelay(ISPub->Period);
	}
}

void ISPublisher_Start(ISPublisher_Struct *ISPub) {
	const osThreadAttr_t ISPub_TaskAttr = { .name =
			"Instant Consumption Publisher", .stack_size = 512 * 4, .priority =
			osPriorityNormal, };
	ISPub->Period = 0;
	ISPub->PubThread = osThreadNew(ISPublisher_Thread, ISPub, &ISPub_TaskAttr);
}

void ISPublisher_EnPubCMD(void *Handler, char *Msg, char *Resp) {
	ISPublisher_Struct *Publisher = (ISPublisher_Struct*) Handler;
	int period = 0;
	uint32_t res = sscanf(Msg, "%d", &period);
	if (res != 1) {
		sprintf(Resp, "-1;");
		return;
	}
	Publisher->Period = period;
	if (period) osThreadResume(Publisher->PubThread);
	sprintf(Resp, "0;");
	return ;
}
