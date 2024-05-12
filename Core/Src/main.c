/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "Servo/Servo.h"
#include "Motor/MotorControl.h"
#include "GPIO/GPIOHandler.h"
#include "Serial_Monitor.h"
#include "SpeedControlDuration/SpeedControlDuration.h"
#include "BNOPublisher/BNOPublisher.h"
#include "InstantConsumption/InstantConsumption.h"
#include "BattPublisher/BattPublisher.h"
#include "DistancePublisher/DistancePublisher.h"
#include "SteeringMotor/SteeringMotor.h"
#include "I2C_Handler/I2C_Handler.h"
#include "VL53_Handler/VL53_Handler.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BATT_ADC_CHANNEL 		ADC_CHANNEL_13
#define INST_ADC_CHANNEL 		ADC_CHANNEL_11
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void SystemReset(void *Handler, char *InMsg, char *Resp);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask",
		.stack_size = 1024 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* USER CODE BEGIN PV */
uint32_t Channels = BATT_ADC_CHANNEL;
//Servo_HandleStruct Servo;
I2C_OS_HandlerStruct i2c_dev = { .hi2c = &hi2c1, };
GPIO_HandlerStruct Blinker = { .Pin = LD2_Pin, .Port = LD2_GPIO_Port };
Steering_HandlerStruct SteerControl =
		{ .htim = &htim3, .Channel = TIM_CHANNEL_2, .Offset = 2,
				.negLimit = -25, .posLimit = 25, };
Motor_HandlerStruct Motor;
ADC_HandleStruct ADC_Handler = (ADC_HandleStruct ) { .hadc = &hadc1, .Channels =
				&Channels, .NumofChannel = 1, };

VL53L0X_Dev_t Vl53l0x[4] = { 0 };

Vl53l0x_HandlerStruct Vl53l0x_Dev_0 = { .XshutPin = { .Port = Xshut0_GPIO_Port,
		.Pin =
		Xshut0_Pin },
//		.XshutPin = {.Port = GPIOC, .Pin = GPIO_PIN_5},
		.Dev =& Vl53l0x[0],
		 .Address = 0x31 << 1 };
Vl53l0x_HandlerStruct Vl53l0x_Dev_1 = { .XshutPin = { .Port = Xshut1_GPIO_Port,
		.Pin =
		Xshut1_Pin },
//		.XshutPin = {.Port = GPIOC, .Pin = GPIO_PIN_6},
		.Dev = &Vl53l0x[1],
		 .Address = 0x32 << 1 };
Vl53l0x_HandlerStruct Vl53l0x_Dev_2 = { .XshutPin = { .Port = Xshut2_GPIO_Port,
		.Pin =
		Xshut2_Pin },
		.Dev = &Vl53l0x[2],.Address = 0x33 << 1 };

Vl53l0x_HandlerStruct Vl53l0x_Dev_3 = { .XshutPin = { .Port = Xshut3_GPIO_Port,
		.Pin =
		Xshut3_Pin },.Dev = &Vl53l0x[3],  .Address = 0x34 << 1 };

SerialMonitor_Struct SerialMon;
UARTHandler_Struct UART = { .RxBufferSize = 100, .huart = &huart2 };
UARTHandler_Struct UART1 = { .RxBufferSize = 100, .huart = &huart1 };

BNOPublisher_Struct BNOPub = (BNOPublisher_Struct ) { .BNO_hi2c = &i2c_dev,
				.Period = 10, .UART_Handler = &UART, };

ISPublisher_Struct ISPub = (ISPublisher_Struct ) { .ADC_Handler = &ADC_Handler,
				.ADC_Channel = INST_ADC_CHANNEL, .Period = 50, .UART_Handler =
						&UART, };

DistPublisher_Struct DistPub = { .i2c_dev = &i2c_dev, .UART_Handler = &UART,
		.Period = 10, };

BattPublisher_Struct BattPub = (BattPublisher_Struct ) { .ADC_Handler =
				&ADC_Handler, .ADC_Channel = BATT_ADC_CHANNEL, .Period = 50,
				.UART_Handler = &UART, };

SpeedControlDuration_HandleStruct SpeedController =
		(SpeedControlDuration_HandleStruct ) { .Motor = &Motor, .Steer =
						&SteerControl, .UART = &UART };
SerialMonitor_CBStruct SerMon_CBTable[] = { { .Function = Motor_setSpeedCMD,
		.ObjHandler = (void*) &Motor }, { .Function = Steering_SetAngleCMD,
		.ObjHandler = (void*) &SteerControl }, { NULL, NULL }, { NULL, NULL }, {
		.Function = BattPublisher_EnPubCMD, .ObjHandler = (void*) &BattPub }, {
		.Function = ISPublisher_EnPubCMD, .ObjHandler = (void*) &ISPub }, {
		.Function = BNOPublisher_EnPubCMD, .ObjHandler = (void*) &BNOPub }, {
		.Function = DistPublisher_EnPubCMD, .ObjHandler = (void*) &DistPub }, {
		.Function = SpeedControlDuration_setVCDCMD, .ObjHandler =
				(void*) &SpeedController }, { .Function = SystemReset,
		.ObjHandler = NULL } };

SerialMonitor_Struct SerMon = { .UART = &UART, .Callback = SerMon_CBTable,
		.CallbackSize = sizeof(SerMon_CBTable) / sizeof(SerMon_CBTable[0]) };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
//	NVIC_SystemReset();
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_TIM3_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_USART1_UART_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
//	uint8_t ret = 0;
//	int i = 0;
//	HAL_GPIO_WritePin(Xshut0_GPIO_Port, Xshut0_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(Xshut1_GPIO_Port, Xshut1_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(Xshut2_GPIO_Port, Xshut2_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(Xshut3_GPIO_Port, Xshut3_Pin, GPIO_PIN_RESET);
////	HAL_GPIO_WritePin(Xshut1_GPIO_Port, Xshut1_Pin, GPIO_PIN_SET);
//	for (i = 0; i < 128; i++) {
//		ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t) i << 1 | 0, 5, 1000);
//		if (ret == HAL_OK) {
//			printf("OK\r\n");
//		}
//	}
//	HAL_GPIO_WritePin(Xshut0_GPIO_Port, Xshut0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Xshut0_GPIO_Port, Xshut0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Xshut1_GPIO_Port, Xshut1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Xshut2_GPIO_Port, Xshut2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Xshut3_GPIO_Port, Xshut3_Pin, GPIO_PIN_RESET);

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 27;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 59999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 27;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 59999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 15, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, Xshut3_Pin | Xshut1_Pin | Xshut0_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Xshut2_GPIO_Port, Xshut2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Xshut3_Pin Xshut1_Pin Xshut0_Pin */
	GPIO_InitStruct.Pin = Xshut3_Pin | Xshut1_Pin | Xshut0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : Xshut2_Pin */
	GPIO_InitStruct.Pin = Xshut2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Xshut2_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart == UART.huart) {
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		UART_RcvToIdle_CB(&UART, Size);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == UART.huart) {
		UART_Rcv_CB(&UART);
	}
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (i2c_dev.hi2c == hi2c) {
		I2C_OS_MEM_RxCpltCB(&i2c_dev);
	}
}
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (i2c_dev.hi2c == hi2c) {
		I2C_OS_MEM_TxCpltCB(&i2c_dev);
	}
}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (i2c_dev.hi2c == hi2c) {
		I2C_OS_MasterTxCpltCB(&i2c_dev);
	}
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (i2c_dev.hi2c == hi2c) {
		I2C_OS_MasterRxCpltCB(&i2c_dev);
	}
}

void SystemReset(void *Handler, char *InMsg, char *Resp) {
	NVIC_SystemReset();
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN 5 */
	UART_Init(&UART, &huart2);
	UART_Init(&UART1, &huart1);
	I2C_OS_Init(&i2c_dev, &hi2c1);
//	ADC_Start_DMA(&ADC_Handler);
	Motor_Init(&Motor, &htim1, TIM_CHANNEL_3);

	SerialMonitor_Start(&SerMon);
	DistPub.NumofSens = 4;
	DistPub.Vl53l0 = pvPortMalloc(
			DistPub.NumofSens * sizeof(Vl53l0x_HandlerStruct));
	DistPub.Vl53l0[0] = Vl53l0x_Dev_0;
	DistPub.Vl53l0[1] = Vl53l0x_Dev_1;
	DistPub.Vl53l0[2] = Vl53l0x_Dev_2;
	DistPub.Vl53l0[3] = Vl53l0x_Dev_3;
//	Servo_Init(&Servo, &htim3, TIM_CHANNEL_2, 25, 0);

	BNOPublisher_Start(&BNOPub);
	DistPublisher_Start(&DistPub);
//	ISPublisher_Start(&ISPub);
//	BattPublisher_Start(&BattPub);
	SpeedControlDuration_Init(&SpeedController);
	Motor_Start(&Motor);
	Steering_Start(&SteerControl);
//	Servo_Start(&Servo);
	Motor_setSpeed(&Motor, 0);
//	St
//	SerialMonitor_Start(&SerialMon);
//	Motor_setSpeed(&Motor, 0);
	UART_printf(&UART, "I am awake!!\r\n");
//	Motor_setSpeed(&Motor, 50);
	/* Infinite loop */
//	int status = 0;
//	for (int i = 0;i < 255;i++) {
//		status = I2C_OS_IsDeviceReady(&i2c_dev, i, 5, 100);
//		if (status == HAL_OK) {
//			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//		}
//	}
	for (;;) {
		GPIO_TogglePin(Blinker);
		osDelay(500);
	}
	/* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM10 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM10) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
