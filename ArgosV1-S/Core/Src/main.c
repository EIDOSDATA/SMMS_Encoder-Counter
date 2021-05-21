/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct _paramType {
	float wheelRadius; // 1
	uint32_t encoderPulseCount; // 2
	float targetDistance; // 3
} __attribute__((aligned(1), packed)) WheelParam;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TARGET_PULSE_NUMBER           1

#define DIVISOR                       1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define MAX_BUFLEN 1024 // used
char buf[MAX_BUFLEN]; // used
int bufHead = 0, bufTail = 0; // used
uint8_t tmpbuf; // UART interrupt buf used

char mainBuf[20] = { 0 }; // data input buf used
char getdata; // get char used

uint8_t cameraflag = 0; // Camera used
uint8_t uartflag = 0; // Uart Action(Camera ADC) used
uint8_t confflag = 0; // Configuration Mode used

WheelParam wP;
int encoderTargetCount = -1;
int adcValue[3]; // ADC DATA SAVE
HAL_StatusTypeDef result;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float diameter(float radius) {
	return (2 * 3.1415 * radius);
}

float rotationForShoot(float targetDistance, float wheelDiameter) {
	return (targetDistance / wheelDiameter);
}

int targetPulseCount(float rotationCount, int encoderPulseCnt) {
	return (int) ((((rotationCount * encoderPulseCnt) / TARGET_PULSE_NUMBER)
			/ DIVISOR));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == huart2.Instance) {
		HAL_UART_Receive_IT(&huart2, &tmpbuf, 1);
		buf[bufTail] = huart->pRxBuffPtr[0];
		bufTail++;
		bufTail %= MAX_BUFLEN;
	}
} // END OF UART2
int _write(int file, unsigned char *p, int len) {
	HAL_UART_Transmit(&huart2, p, len, 100);
	return len;
}
/*
 int __io_putchar(int ch) {
 uint8_t *tr = (uint8_t*) &ch;
 HAL_UART_Transmit(&huart2, &tr[0], 1, -1);
 return ch;
 }
 */
int __io_getchar() {
	register int ret;

	__retry: if (bufHead != bufTail) {
		ret = buf[bufHead];
		if (ret == '\r' || ret == '\n') {
			ret = '\n';
		}
		bufHead++;
		bufHead %= MAX_BUFLEN;
	} else {
		goto __retry;
		//return -1;
	}
	return ret;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) // GPIO INTERRUPT
{
	if (GPIO_Pin == GPIO_PIN_6) //
	{
		HAL_GPIO_WritePin(MARK2_GPIO_Port, MARK2_Pin, SET);
	} else if (GPIO_Pin == ENCODER_INT_Pin) // ENCODER INTERRUPT
	{
		HAL_ADC_Start_DMA(&hadc1, adcValue, 3);
		for (int i = 0; i < 3; i++) { // ADC data save
			HAL_DMA_PollForTransfer(&hdma_adc1, HAL_DMA_FULL_TRANSFER, 100); // ADC DATA GET 3
		}
		HAL_ADC_Stop_DMA(&hadc1);
		printf("light value : ADC = %04d   %04d   %04d\r\n", adcValue[0],
				adcValue[1], adcValue[2]);
	}
}
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

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2, &tmpbuf, 1);
	printf(
			"Hello Commander! Activate Camera : s   Car Configuration : c\r\n\r\n");
	encoderTargetCount = targetPulseCount(
			rotationForShoot(wP.targetDistance, diameter(wP.wheelRadius)),
			wP.encoderPulseCount);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		HAL_UART_Receive_IT(&huart2, &tmpbuf, 1);
		getdata = getchar();

		if ((getdata == 's' || getdata == 'S') && confflag == 0) { // UART INPUT 'S'
			printf("Camera activated\r\n");
			HAL_GPIO_WritePin(AUTOFOCUS_GPIO_Port, AUTOFOCUS_Pin, SET);
			HAL_GPIO_WritePin(SHUTTER_GPIO_Port, SHUTTER_Pin, SET);
			HAL_GPIO_WritePin(MARK2_GPIO_Port, MARK2_Pin, RESET);
			cameraflag = 1;
			uartflag = 1;
			if (cameraflag == 1) { // Camera Action
				HAL_Delay(10);
				HAL_GPIO_WritePin(AUTOFOCUS_GPIO_Port, AUTOFOCUS_Pin, RESET);
				HAL_GPIO_WritePin(SHUTTER_GPIO_Port, SHUTTER_Pin, RESET);
				cameraflag = 0;
			}
			if (uartflag == 1) {
				HAL_ADC_Start_DMA(&hadc1, adcValue, 3);
				for (int i = 0; i < 3; i++) { // ADC data save
					HAL_DMA_PollForTransfer(&hdma_adc1, HAL_DMA_FULL_TRANSFER,
							100); // ADC DATA GET 3
				}
				HAL_ADC_Stop_DMA(&hadc1);
				printf("light value : ADC = %04d   %04d   %04d\r\n",
						adcValue[0], adcValue[1], adcValue[2]);
				uartflag = 0;
			}
		}
		if ((getdata == 'c' || getdata == 'C') && confflag == 0) {
			printf("Car configuration activated. Plz input data\r\n");
			printf(
					"R.Radius\r\nE.Encoder pulse count\r\nT.Target Distance\r\nF.Forwarding Data\r\nV.View\r\nx.EXIT\r\n");
			confflag = 1;
		}
		// Command Mode Index
		if (confflag == 1) {
			switch (getdata) { // UART INPUT 'C' // Configuration Mode
			case 'r': // Car radius
			case 'R':
				printf("\r\nRadius data input\r\n");
				gets(mainBuf); // Read Data
				printf("Input Data : %s / ArrSize : %d\r\n", mainBuf,
						sizeof(wP)); // Print Data
				wP.wheelRadius = atof(mainBuf);
				encoderTargetCount = targetPulseCount(
						rotationForShoot(wP.targetDistance,
								diameter(wP.wheelRadius)),
						wP.encoderPulseCount);
				break;
			case 'e': // Encoder pluse
			case 'E':
				printf("\r\nEncoder pulse count input\r\n");
				gets(mainBuf); // Read Data
				printf("Input Data : %s / ArrSize : %d\r\n", mainBuf,
						sizeof(wP)); // Print Data
				wP.encoderPulseCount = atoi(mainBuf);
				if (wP.encoderPulseCount == 0) {
					break;
				}
				encoderTargetCount = targetPulseCount(
						rotationForShoot(wP.targetDistance,
								diameter(wP.wheelRadius)),
						wP.encoderPulseCount);
				break;
			case 't': // Target distance
			case 'T':
				printf("\r\nTarget Distance input\r\n");
				gets(mainBuf); // Read Data
				printf("Input Data : %s / ArrSize : %d\r\n", mainBuf,
						sizeof(wP)); // Print Data
				wP.targetDistance = atof(mainBuf);
				encoderTargetCount = targetPulseCount(
						rotationForShoot(wP.targetDistance,
								diameter(wP.wheelRadius)),
						wP.encoderPulseCount);
				break;
			case 'f': // Data Save
			case 'F':
				printf("\r\nForwarding data\r\n");
				HAL_SPI_Transmit(&hspi1, (uint8_t*) &wP, sizeof(wP), -1);
				printf("Activate Camera : s   Car Configuration : c\r\n\r\n");
				confflag = 0;
				break;
			case 'v': // Data View
			case 'V':
				printf(
						"\r\nEncoderPulseCnt = %lu\r\nWheel Radius = %0.5f\r\nTrigger Distance = %0.2f\r\nTarget pulse count = %d\r\n",
						wP.encoderPulseCount, wP.wheelRadius, wP.targetDistance,
						encoderTargetCount);
				break;
			case 'x': // EXIT
			case 'X':
				printf("\r\nINPUT MODE CLOSE\r\n");
				printf("Activate Camera : s   Car Configuration : c\r\n\r\n");
				confflag = 0;
				break;
			} // End of Switch Case
		} // End of If
	} // End of While
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
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
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
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
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
	/* USART2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	/* EXTI15_10_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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
	hadc1.Init.Resolution = ADC_RESOLUTION_10B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 3;
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
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = 3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

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

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
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
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

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
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	/* DMA2_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			MARK2_Pin | MARK3_Pin | AUTOFOCUS_Pin | SHUTTER_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : MARK2_Pin MARK3_Pin */
	GPIO_InitStruct.Pin = MARK2_Pin | MARK3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PC6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : ENCODER_INT_Pin */
	GPIO_InitStruct.Pin = ENCODER_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ENCODER_INT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : AUTOFOCUS_Pin SHUTTER_Pin */
	GPIO_InitStruct.Pin = AUTOFOCUS_Pin | SHUTTER_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
