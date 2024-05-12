/*
 * BattPublisher.c
 *
 *  Created on: Jan 23, 2024
 *      Author: nhakh
 */
#include "BattPublisher.h"

void BattPublisher_Thread(void *arg) {
	BattPublisher_Struct *BattPub = (BattPublisher_Struct*) arg;
	uint32_t raw = 0;
	while (1) {
		if (!BattPub->Period)
			osThreadSuspend(BattPub->PubThread);
		raw = 0;
		for (int i = 0; i < 100; i++) {

			raw += ADC_GetValueFromeChannel(BattPub->ADC_Handler,
					BattPub->ADC_Channel);
		}
		raw /= 100;
		BattPub->value = 1.0 * raw / 4095 * 30.65;
		UART_printf(BattPub->UART_Handler, "@5:%.3f;;\r\n", BattPub->value);
		osDelay(BattPub->Period);
	}
}

void BattPublisher_Start(BattPublisher_Struct *BattPub) {
	const osThreadAttr_t BattPub_TaskAttr = { .name =
			"Instant Consumption Publisher", .stack_size = 512 * 4, .priority =
			osPriorityNormal, };
	BattPub->Period = 0;
	BattPub->PubThread = osThreadNew(BattPublisher_Thread, BattPub,
			&BattPub_TaskAttr);
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
	if (period)
		osThreadResume(Publisher->PubThread);
	sprintf(Resp, "0;");
	return;
}

