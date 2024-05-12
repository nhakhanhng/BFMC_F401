/*
 * ADCHandler.h
 *
 *  Created on: Jan 23, 2024
 *      Author: nhakh
 */

#ifndef ADC_ADCHANDLER_H_
#define ADC_ADCHANDLER_H_
#include "main.h"


typedef struct {
	ADC_HandleTypeDef *hadc;
	uint32_t *Channels;
	uint32_t *Raw_val;
	uint8_t NumofChannel;
} ADC_HandleStruct;

void ADC_Start(ADC_HandleStruct *ADC_Handler);
void ADC_Start_DMA(ADC_HandleStruct *ADC_Handler);
uint32_t ADC_GetValueFromeChannel(ADC_HandleStruct *ADC_Handler,uint32_t Channel);


#endif /* ADC_ADCHANDLER_H_ */
