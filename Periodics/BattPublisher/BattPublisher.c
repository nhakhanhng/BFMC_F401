/*
 * BattPublisher.c
 *
 *  Created on: Jan 23, 2024
 *      Author: nhakh
 */
#include "BattPublisher.h"

void BattPublisher_Thread(void *arg) {
	BattPublisher_Struct *BattPub = (BattPublisher_Struct*) arg;

	while (1) {
		if (!BattPub->Period)
			osThreadSuspend(BattPub->PubThread);
		uint32_t raw = ADC_GetValueFromeChannel(BattPub->ADC_Handler,
				BattPub->ADC_Channel);
		BattPub->value = 1.0 * raw / 7358.54;
		UART_printf(BattPub->UART_Handler, "@5:%.3f;;\r\n", BattPub->value);
		osDelay(BattPub->Period);
	}
}

void BattPublisher_Start(BattPublisher_Struct *BattPub) {
	const osThreadAttr_t BattPub_TaskAttr = { .name =
			"Instant Consumption Publisher", .stack_size = 512 * 4, .priority =
			osPriorityNormal, };
	BattPub->Period = 0;
	BattPub->PubThread = osThreadNew(BattPublisher_Thread, BattPub, &BattPub_TaskAttr);
}

void BattPublisher_EnPubCMD(void *Handler, char *Msg, char *Resp) {
	BattPublisher_Struct *Publisher = (BattPublisher_Struct*) Handler;
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


