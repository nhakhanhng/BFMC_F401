/*
 * ADCHandler.c
 *
 *  Created on: Jan 23, 2024
 *      Author: nhakh
 */
#include "ADCHandler.h"


void ADC_Start(ADC_HandleStruct *ADC_Handler) {
	HAL_ADC_Start(ADC_Handler->hadc);
}
void ADC_Start_DMA(ADC_HandleStruct *ADC_Handler) {
	HAL_ADC_Start_DMA(ADC_Handler->hadc, ADC_Handler->Raw_val, ADC_Handler->NumofChannel);
}
uint16_t ADC_GetValueFromeChannel(ADC_HandleStruct *ADC_Handler,uint32_t Channel) {
	for (uint8_t i = 0;i < ADC_Handler->NumofChannel;i++) {
		if (Channel == ADC_Handler->Channel[i]) {
			return ADC_Handler->Raw_val[i];
		}
	}
	return 0;
}
