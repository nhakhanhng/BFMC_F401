#include "GPIOHandler.h"

//GPIO_HandlerStruct PA10 = {.Port = GPIOA,.Pin = GPIO_PIN_10};
//GPIO_HandlerStruct PB5 = {.Port = GPIOB, .Pin = GPIO_PIN_5};
//GPIO_HandlerStruct PC6 = {.Port = GPIOC, .Pin = GPIO_PIN_6};

void GPIO_TogglePin(GPIO_HandlerStruct GPIO)
{
	return HAL_GPIO_TogglePin(GPIO.Port, GPIO.Pin);
}

void GPIO_WritePin(GPIO_HandlerStruct GPIO, uint32_t PinState)
{
	return HAL_GPIO_WritePin(GPIO.Port, GPIO.Pin, PinState);
}
