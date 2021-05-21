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
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct wheel_param
{
	uint32_t one_rot_enc_pls; // 1 rotation encoder pulse value >> 2000
	float car_w_rad; // Car Wheel Radius >> 0.275(m)
	float tgt_dist; // Target Distance (Camera Action) >> 5.0(m)
} __attribute__((aligned(1), packed)) wheel_param_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Setting var at page 63
#define BACKUP_FLASH_ADDR 0x0801F800

#define TARGET_PULSE_NUMBER           1

#define DIVISOR                       1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int a_pls_cnt = 0;
int b_pls_cnt = 0;

uint8_t run_f = 0;
uint8_t cmd_f = 0;
uint8_t enterkey_f = 0;
extern char usb_rx_buf[200];
char usb_tx_buf[300];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
	case GPIO_PIN_5:
		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_0);
		if (run_f)
		{
			a_pls_cnt++;
		}
		break;
	case GPIO_PIN_6:
		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_1);
		if (run_f)
		{
			b_pls_cnt++;
		}
		break;
	case GPIO_PIN_7:
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
		//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,SET);
		// = true;
		break;
	}
	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,RESET);
}

float diameter(float radius)
{
	return (2 * 3.1415 * radius);
}

float n_of_rot_to_photo_dist(float tgt_dist, float w_diameter)
{
	return (tgt_dist / w_diameter);
}

int tgt_pls_cnt(float w_turn_enc_pls, int enc_pls_f)
{
	return (int) ((((w_turn_enc_pls * enc_pls_f) / TARGET_PULSE_NUMBER)
			/ DIVISOR));
}

void save_w_param(wheel_param_t *wp)
{
	HAL_FLASH_Unlock();
	{
		FLASH_EraseInitTypeDef fler;
		uint32_t perr;
		fler.TypeErase = FLASH_TYPEERASE_PAGES;
		fler.Banks = 1;
		fler.Page = 63;
		fler.NbPages = 1;
		HAL_FLASHEx_Erase(&fler, &perr);
		register uint64_t *_targetAddr = (uint64_t*) (wp);
		for (uint8_t i = 0; i <= (sizeof(wheel_param_t) * 2); i +=
				sizeof(uint64_t))
		{
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
			BACKUP_FLASH_ADDR + i, _targetAddr[i / sizeof(uint64_t)]);
		}
	}
	HAL_FLASH_Lock();
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	wheel_param_t wp;
	//At 1st, memcpy from backup addr

	memcpy(&wp, (void*) (BACKUP_FLASH_ADDR), sizeof(wheel_param_t));

	//2nd, compare memory
	/*if (wp.encoderPulseCount == 0xFFFFFFFF)
	 {
	 //if flash not initialized, set value to default
	 wp.encoderPulseCount = 2000;
	 wp.targetDistance = 5.0;
	 wp.wheelRadius = 0.324;
	 SaveWheelParam(&wp);
	 }*/

	int enc_val_for_photo_dist = -1;
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
	MX_TIM3_Init();
	MX_USB_Device_Init();
	/* USER CODE BEGIN 2 */
	LL_TIM_EnableIT_UPDATE(TIM3);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	HAL_Delay(100);
	enc_val_for_photo_dist = tgt_pls_cnt(
			n_of_rot_to_photo_dist(wp.tgt_dist, diameter(wp.car_w_rad)),
			wp.one_rot_enc_pls);
	cmd_f = 0;
	while (1)
	{
		switch (cmd_f)
		{
		case 1:	//set wheel param
			CDC_Transmit_FS((uint8_t*) "Input wheel radius.. > ", 23);
			while (!enterkey_f)
				;
			enterkey_f = 0;
			wp.car_w_rad = atof(usb_rx_buf);
			enc_val_for_photo_dist = tgt_pls_cnt(
					n_of_rot_to_photo_dist(wp.tgt_dist, diameter(wp.car_w_rad)),
					wp.one_rot_enc_pls);
			cmd_f = 0;
			break;
		case 2:	//set encoder pulse count;
			CDC_Transmit_FS((uint8_t*) "Input encoder pulse cnt.. > ", 28);
			while (!enterkey_f)
				;
			enterkey_f = 0;
			wp.one_rot_enc_pls = atoi(usb_rx_buf);
			if (wp.one_rot_enc_pls == 0)
			{
				break;
			}
			enc_val_for_photo_dist = tgt_pls_cnt(
					n_of_rot_to_photo_dist(wp.tgt_dist, diameter(wp.car_w_rad)),
					wp.one_rot_enc_pls);
			cmd_f = 0;
			break;
		case 3:	//set trigger distance
			CDC_Transmit_FS((uint8_t*) "Input trigger distance(m) > ", 28);
			while (!enterkey_f)
				;
			enterkey_f = 0;
			wp.tgt_dist = atof(usb_rx_buf);
			enc_val_for_photo_dist = tgt_pls_cnt(
					n_of_rot_to_photo_dist(wp.tgt_dist, diameter(wp.car_w_rad)),
					wp.one_rot_enc_pls);
			cmd_f = 0;
			break;
		case 4:
			sprintf(usb_tx_buf, "EncoderPulseCnt= %lu\r\n"
					"Wheel Radius= %0.5f\r\n"
					"Trigger Distance= %0.2f\r\n"
					"Target pulse count= %d\r\n"
					"Auto triggering= %s\r\n", wp.one_rot_enc_pls, wp.car_w_rad,
					wp.tgt_dist, enc_val_for_photo_dist,
					(run_f == 1) ? ("Started") : ("Disabled"));
			CDC_Transmit_FS((uint8_t*) usb_tx_buf, strlen(usb_tx_buf));
			cmd_f = 0;
			break;
		case 5:
			CDC_Transmit_FS((uint8_t*) "Saving current param...\r\n", 25);
			save_w_param(&wp);
			cmd_f = 0;
			break;
		default:
			cmd_f = 0;
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (run_f && a_pls_cnt >= enc_val_for_photo_dist)
		{
			a_pls_cnt = 0;
			b_pls_cnt = 0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
			LL_TIM_ClearFlag_UPDATE(TIM3);
			LL_TIM_EnableCounter(TIM3);
			char __buf = 0xEE;
			//CDC_Transmit_FS(&__buf, 1);
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
	RCC_OscInitStruct.PLL.PLLN = 42;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the peripherals clocks
	 */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
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
