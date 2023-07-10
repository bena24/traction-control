/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "throttle_control.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//Trac_Control tc1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_DAC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct tim1 {
	int unsigned Is_First_Captured;
	unsigned long IC_Value1;
	unsigned long IC_Value2;
	unsigned long Difference;
	unsigned long Frequency;
	int unsigned CalculationOK;
} tim1;
struct tim2 {
	int unsigned Is_First_Captured;
	unsigned long IC_Value1;
	unsigned long IC_Value2;
	unsigned long Difference;
	unsigned long Frequency;
	int unsigned CalculationOK;
} tim2;
struct tim3 {
	int unsigned Is_First_Captured;
	unsigned long IC_Value1;
	unsigned long IC_Value2;
	unsigned long Difference;
	unsigned long Frequency;
	int unsigned CalculationOK;
} tim3;

uint8_t send[4] = { };
uint8_t receive[20] = { };
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
	//dMX_I2C1_Init();
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	//MX_TIM2_Init();
	//MX_TIM3_Init();
	//MX_TIM3_Init();
	//MX_TIM11_Init();
	//MX_TIM10_Init();
	//HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	//HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	//HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
	//HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	//MX_I2C1_Init();

	//htim3.Channel = HAL_TIM_ACTIVE_CHANNEL_1;
	//htim2.Channel = HAL_TIM_ACTIVE_CHANNEL_1;
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_DAC_Init();
	MX_ADC1_Init();
	MX_TIM5_Init();
	MX_I2C1_Init();
	MX_ADC2_Init();
	MX_ADC3_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	//uint32_t var = 0;
	//uint32_t var = (uint32_t)(val*4096)/3.3;
	//htim13.Channel = HAL_TIM_ACTIVE_CHANNEL_1;
	htim5.Channel = HAL_TIM_ACTIVE_CHANNEL_2;
	//htim2.Channel = HAL_TIM_ACTIVE_CHANNEL_1;
	//HAL_TIM_IC_Start_IT(&htim13, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_ADC_Start(&hadc1);
	//HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
	//Trac_Control_Init(&tc1);
	HAL_DAC_Start(&hdac, 1);
	HAL_I2C_EnableListen_IT(&hi2c1);
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//value = HAL_ADC_GetValue(&hadc1);
		//printf("%lu\n\r", value);
		//HAL_Delay(100);
		//HAL_ADC_Stop(&hadc1);
		//HAL_TIM_IC_CaptureCallback(&htim13);
		htim2.Channel = HAL_TIM_ACTIVE_CHANNEL_1;
		//HAL_TIM_IC_CaptureCallback(&htim2);
		htim2.Channel = HAL_TIM_ACTIVE_CHANNEL_3;
		//HAL_TIM_IC_CaptureCallback(&htim2);
		//HAL_TIM_IC_CaptureCallback(&htim5);
		//HAL_TIM_IC_CaptureCallback(&htim3);
		//HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
		//printf("Value = %lu", IC_Value1);
		if (tim1.CalculationOK == 1)
			printf(
					" TIM2: Period = %lu ---- Frequency = %lu ---- PCLK1 frequency %lu \n\r",
					tim1.Difference, tim1.Frequency, HAL_RCC_GetPCLK1Freq());
		if (tim2.CalculationOK == 1) {
			printf(
					" TIM5: Period = %lu ---- Frequency = %lu ---- PCLK1 frequency %lu \n\r",
					tim2.Difference, tim2.Frequency, HAL_RCC_GetPCLK1Freq());
		}
		if (tim3.CalculationOK == 1) {
			printf(
					" TIM2 (2): Period = %lu ---- Frequency = %lu ---- PCLK1 frequency %lu \n\r",
					tim3.Difference, tim3.Frequency, HAL_RCC_GetPCLK1Freq());
		}
		tim1.CalculationOK = 0;
		tim2.CalculationOK = 0;
		tim3.CalculationOK = 0;
		//printf("MPH = %f\n\r", Frequency_to_MPH(tim3.Frequency));
		//HAL_TIM_IC_CaptureCallback(&htim3);
		//*MPH = (uint8_t) Frequency_to_MPH(Frequency);
		//HAL_I2C_Master_Transmit(&hi2c1, address<<1, (uint8_t*)buffer, sizeof(buffer), 50);
		//HAL_I2C_Slave_Receive(&hi2c1, buffer, sizeof(buffer), 50);
		//HAL_I2C_Slave_Transmit_IT(&hi2c1, buf, sizeof(buf));
		//if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_BUSY_RX)
		//HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, buffer, sizeof(buffer), I2C_NEXT_FRAME);
		//HAL_I2C_Slave_Seq_Transmit_IT (&hi2c1, buf, sizeof(buf), I2C_FIRST_FRAME);
		//HAL_I2C_EnableListen_IT(&hi2c1);
		//HAL_I2C_AddrCallback(&hi2c1, 1, 0x69);
		/*for (int i = 0; i < sizeof(receive); ++i)
		 printf("%u ", receive[i]);
		 printf("\n\r");*/
		/*if (HAL_I2C_IsDeviceReady(&hi2c1, address<<1, 10, 100) == HAL_OK)
		 printf("Device READY\n\r");
		 else
		 printf("NOT READY\n\r");*/
		//++nLoop;
		//printf("nLoop == %d \n\r", nLoop);
		/*
		 var = val*(4096)/3.3;
		 HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,
		 DAC_ALIGN_12B_R, var);
		 val += 0.5;
		 HAL_Delay(1);
		 if (value>3)
		 value=0.2;
		 */
		//HAL_Delay(500);
		/*tc1.throttle_percent = Analog_to_throttle(HAL_ADC_GetValue(&hadc1));
		 if (tc1.throttle_percent > tc1.max_throttle)
		 tc1.throttle_percent = tc1.max_throttle;
		 if (tim1.CalculationOK && tim2.CalculationOK && tim3.CalculationOK){
		 tc1.Front_Speed = Frequency_to_MPH(Front_Wheel_Average(tim2.Frequency, tim1.Frequency));
		 tc1.Rear_Speed = Frequency_to_MPH(tim3.Frequency);
		 if (tc1.off != 1 && tc1.Front_Speed > 0)
		 tc1.throttle_percent = Trac_Control_Run(&tc1);
		 else if (tc1.time != 0 || tc1.prev_error != 0.0)
		 Reset_Pid(&tc1);
		 }
		 HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Throttle_to_analog(tc1.throttle_percent));


		 ++send[0];// = (uint8_t) i; //tc1.Front_Speed;
		 send[1] = (uint8_t) tc1.max_throttle;
		 ++send[2];// = (uint8_t) i; //tc1.throttle_percent
		 send[3] = tc1.running;
		 Slip_Target_Change(&tc1, receive[0]);
		 Max_Throttle_Change(&tc1, receive[1]);
		 if (send[0] > 50)
		 send[0] = 0;
		 if (send[2] > 150)
		 send[0] = 0;*/
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_LSE, RCC_MCODIV_1);
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
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void) {

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.ScanConvMode = DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void) {

	/* USER CODE BEGIN ADC3_Init 0 */

	/* USER CODE END ADC3_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC3_Init 1 */

	/* USER CODE END ADC3_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc3.Init.Resolution = ADC_RESOLUTION_12B;
	hadc3.Init.ScanConvMode = DISABLE;
	hadc3.Init.ContinuousConvMode = DISABLE;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DMAContinuousRequests = DISABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc3) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC3_Init 2 */

	/* USER CODE END ADC3_Init 2 */

}

/**
 * @brief DAC Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC_Init(void) {

	/* USER CODE BEGIN DAC_Init 0 */

	/* USER CODE END DAC_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC_Init 1 */

	/* USER CODE END DAC_Init 1 */
	/** DAC Initialization
	 */
	hdac.Instance = DAC;
	if (HAL_DAC_Init(&hdac) != HAL_OK) {
		Error_Handler();
	}
	/** DAC channel OUT1 config
	 */
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DAC_Init 2 */

	/* USER CODE END DAC_Init 2 */

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
	hi2c1.Init.OwnAddress1 = 210;
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
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_IC_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4294967295;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin : PA8 */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		if (tim1.Is_First_Captured == 0) {
			tim1.IC_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			tim1.Is_First_Captured = 1;
		} else if (tim1.Is_First_Captured) {
			tim1.IC_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			if (tim1.IC_Value2 > tim1.IC_Value1) {
				tim1.Difference = tim1.IC_Value2 - tim1.IC_Value1;
				tim1.Frequency = HAL_RCC_GetPCLK1Freq() / tim1.Difference;
				tim1.CalculationOK = 1;
			} else
				tim1.CalculationOK = 0;
			tim1.Is_First_Captured = 0;
		}
	}
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		if (tim2.Is_First_Captured == 0) {
			tim2.IC_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			tim2.Is_First_Captured = 1;
		} else if (tim2.Is_First_Captured) {
			tim2.IC_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			if (tim2.IC_Value2 > tim2.IC_Value1) {
				tim2.Difference = tim2.IC_Value2 - tim2.IC_Value1;
				tim2.Frequency = HAL_RCC_GetPCLK1Freq() / tim2.Difference;
				tim2.CalculationOK = 1;
			} else
				tim2.CalculationOK = 0;
			tim2.Is_First_Captured = 0;
		}
	}
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
		if (tim3.Is_First_Captured == 0) {
			tim3.IC_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			tim3.Is_First_Captured = 1;
		} else if (tim3.Is_First_Captured) {
			tim3.IC_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			if (tim3.IC_Value2 > tim3.IC_Value1) {
				tim3.Difference = tim3.IC_Value2 - tim3.IC_Value1;
				tim3.Frequency = HAL_RCC_GetPCLK1Freq() / tim3.Difference;
				tim3.CalculationOK = 1;
			} else
				tim3.CalculationOK = 0;
			tim3.Is_First_Captured = 0;
		}
	}
}

int __io_putchar(int ch) {
	uint8_t c[1];
	c[0] = ch & 0x00FF;
	HAL_UART_Transmit(&huart2, &*c, 1, 10);
	return ch;
}

int _write(int file, char *ptr, int len) {
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		__io_putchar(*ptr++);
	}
	return len;
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
	HAL_I2C_EnableListen_IT(&hi2c1); // Restart
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection,
		uint16_t AddrMatchCode) {
	UNUSED(AddrMatchCode);
	//I2C_NEXT_FRAME
	//I2C_LAST_FRAME
	if (TransferDirection == I2C_DIRECTION_TRANSMIT)
		HAL_I2C_Slave_Sequential_Receive_IT(&hi2c1, receive, sizeof(receive),
				I2C_NEXT_FRAME);
	else {
		HAL_I2C_Slave_Sequential_Transmit_IT(&hi2c1, send, sizeof(send),
				I2C_NEXT_FRAME);
	}

}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
}

/* USER CODE END 4 */

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

