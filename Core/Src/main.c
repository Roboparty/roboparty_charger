
// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 zz020

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "i2c_soft.h"
#include "tmp100.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "parse_command.h"

#include "tws_bms.h"
#include "spi_flash_w25q128.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t hostData[128] = {0};
uint8_t rs485Data[128] = {0};
uint8_t UART1_RX_DMA_IDLE;
uint8_t UART3_RX_DMA_IDLE;
uint8_t ADC1_IDLE = 0;

void hostCmdProcess(uint8_t * cmd, uint16_t len);

float CHRG_I = 0.000f, CHRG_V = 0.000f, BAT_V = 0.000f;
uint16_t adc_buffer0[3] = {0};
uint16_t adc_buffer1[3] = {0};
uint8_t ADC_BUF_SEL = 0;
uint16_t CHRG_PWR = 0;

env_secure_t * pEnvSecure;

char     str_chrg_i[16], str_chrg_v[16], str_bat_v[16];
char     max_current[16];
char     strBmsSoc[6];
char     strBmsSoh[6];
uint16_t BmsSoc[1] = {0};
uint16_t BmsSoh[1] = {0};
char     strTemp[16];
uint8_t  env_est_idx = 0;
float    mid_bat_volt = 0.0f;
float    mid_chrg_current = 0.0f;
float    mid_chrg_volt = 0.0f;
float    mid_temp = 0.0f;
float    temp = 0.0f;

uint8_t  BAT_INST = 0;
uint8_t  BMS_COM_ERR;
uint8_t  CLEAN_EXCP = 0;
uint8_t  CHRG_FULL;
uint8_t  led_blink_cnt = 0;
uint8_t  RED_LED_FLAG = 0;

// ??? printf ? USART3(???? huart3)
int fputc(int ch, FILE *f)
{
    // ????????
    HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END 0 */

void EnvSecureOp()
{
    if((pEnvSecure->estimate_chrg_volt >= 32.0f) || (pEnvSecure->estimate_chrg_volt >= 5.5f && pEnvSecure->estimate_chrg_volt <= 16.0f) ||
       (pEnvSecure->estimate_bat_volt >= 55.0f) || (pEnvSecure->estimate_temp >= 80.0f) ||
       (pEnvSecure->estimate_chrg_current >= 6.0f) 
       )
    {
        g_config.chg_en = 0;
        g_config.ictrl  = 0;
        pEnvSecure->exception = 1;
        ApplyConfig();
    }
    else
    {
        /* Auto-clear exception when fault conditions are no longer present */
        if(pEnvSecure->exception == 1)
        {
            pEnvSecure->exception = 0;
            CLEAN_EXCP = 0;
        }
    }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

	/* USER CODE BEGIN 1 */
//    SCB->VTOR = 0x08004000;
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
	MX_ADC1_Init();

    MX_SPI2_Init();
	MX_TIM1_Init();
	MX_USART3_UART_Init();
	MX_USART1_UART_Init();
    MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start_IT(&htim4);

	if(LoadConfigFromFlash(&g_config) == 1)// No Saved Config data
	{
		g_config.b = 10;
		g_config.g = 10;
		g_config.r = 10;
		g_config.chg_en = 0;
		g_config.ictrl = 0;
        g_config.fan = 49;

		printf("Flash Empty, load default parameter");
	}
	else
	{
//		printf("Flash Cfg\r\nB:%d;G:%d;R:%d;I_CTRL:%d", g_config.b, g_config.g, g_config.r, g_config.ictrl);
	}
	ApplyConfig();
	HAL_Delay(1000);
	I2C_SOFT_INIT();

    pEnvSecure = malloc(sizeof(env_secure_t));

    /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 49);//FAN
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	printf("%s;", "RoboParty USBC Back Board.");

	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, hostData, 128);

	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer0, 3) != HAL_OK)
	{
		Error_Handler();
	}

    if(BAT_INST && CHRG_V >= 4.0f)
    {
        if(ReadBMS_SOC(BmsSoc))
            BMS_COM_ERR = 1;
        else
            sprintf(strBmsSoc, "%d", BmsSoc[0]);
        HAL_Delay(100);
        if(ReadBMS_SOH(BmsSoh))
            BMS_COM_ERR = 1;
        else
            sprintf(strBmsSoh, "%d", BmsSoh[0]);
    }
    else
    {
        BmsSoc[0] = 0;
        BmsSoh[0] = 0;
    }
	while (1)
	{
		if(UART3_RX_DMA_IDLE)
		{

			UART3_RX_DMA_IDLE = 0;
			uint16_t recv_len = 128 - huart3.RxXferCount;
			parse_command(hostData, recv_len);
		}
		if(hadc1.DMA_Handle->State == HAL_DMA_STATE_READY)
		{
			ADC1_IDLE = 0;
            CHRG_I = (float)adc_buffer0[0]/4095.f*3.3/0.4536;
			BAT_V = (float)adc_buffer0[1]/4095.f*3.3/0.0449;
			CHRG_V  = (float)adc_buffer0[2]/4095.f*3.3/0.0909;
            CHRG_PWR = CHRG_I * CHRG_V;
            sprintf(str_chrg_i, "%-3.1fA", CHRG_I);
            sprintf(str_chrg_v, "%-4.1fV", CHRG_V);
            sprintf(str_bat_v, "BAT_V: %-4.1fV", BAT_V);
            temp = TMP100_GetTemperature();
            sprintf(strTemp, "%-4.1foC", temp);

            mid_bat_volt += BAT_V;
            mid_chrg_current += CHRG_I;
            mid_chrg_volt += CHRG_V;
            mid_temp += temp;

            if(env_est_idx >= 2)
            {
                env_est_idx = 0;
                pEnvSecure->estimate_pre_chrg_volt = pEnvSecure->estimate_chrg_volt;
                pEnvSecure->estimate_bat_volt = mid_bat_volt / 3;
                pEnvSecure->estimate_chrg_current = mid_chrg_current / 3;
                pEnvSecure->estimate_chrg_volt = mid_chrg_volt / 3;
                pEnvSecure->estimate_temp = mid_temp / 3;
                mid_bat_volt = 0.0f;
                mid_chrg_current = 0.0f;
                mid_chrg_volt = 0.0f;
                mid_temp = 0.0f;
            }
            else
            {
                env_est_idx ++;
            }

			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer0, 3);
		}

        if(CHRG_V >= 16.0f && g_config.chg_en == 1)
        {
            if(pEnvSecure->estimate_temp <= 50.0f)
                g_config.fan = 49;
            else if((pEnvSecure->estimate_temp > 50.0f) && (pEnvSecure->estimate_temp <= 70.0f))
                g_config.fan = 49 + (pEnvSecure->estimate_temp-50) * 2.5;
            else
                g_config.fan = 99;
        }
        else
            g_config.fan = 0;

        if(CHRG_V >= 16.0f && BAT_INST && pEnvSecure->exception == 0)
        {
            g_config.chg_en = 1;
        }
        else
        {
            g_config.ictrl = 0;
            g_config.chg_en = 0;
        }

        ApplyConfig();

        if(BAT_INST && CHRG_V >= 4.0f)
        {
            if(ReadBMS_SOC(BmsSoc) == 0)
                sprintf(strBmsSoc, "%d", BmsSoc[0]);
            else
                BMS_COM_ERR = 1;
            HAL_Delay(100);
            if(ReadBMS_SOH(BmsSoh) == 0)
                sprintf(strBmsSoh, "%d", BmsSoh[0]);
            else
                BMS_COM_ERR = 1;
            HAL_Delay(100);
        }
        else
            BmsSoc[0] = 0;

        EnvSecureOp();
        if(pEnvSecure->exception == 1 && CLEAN_EXCP == 0)
        {
            CLEAN_EXCP = 1;
            BmsSoc[0] = 0;
            printf("est_chrg_v: %-4.1f\t", pEnvSecure->estimate_chrg_volt);
            printf("est_bat_v: %-4.1f\t", pEnvSecure->estimate_bat_volt);
            printf("est_temp: %-3.1f\t", pEnvSecure->estimate_temp);
            printf("est_chrg_i: %-2.1f\n", pEnvSecure->estimate_chrg_current);
        }

        /* ---- LED Status Indication ---- */
        if(pEnvSecure->exception == 1)
        {
            /* Exception: RED_LED blinks, GREEN_LED off */
            if(led_blink_cnt == 0x03)
            {
                led_blink_cnt = 0;
                RED_LED_FLAG = ~RED_LED_FLAG;
            }
            else
            {
                led_blink_cnt++;                
            }
            if(RED_LED_FLAG)
                HAL_GPIO_WritePin(LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
                
            HAL_GPIO_WritePin(LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
        }
        else if(g_config.chg_en == 1 && CHRG_V >= 16.0f)
        {
            /* Charging: GREEN_LED on, RED_LED on */
            if(BmsSoc[0] != 100)
            {
//                HAL_GPIO_WritePin(LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);//
                HAL_GPIO_WritePin(LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);//
            }
            else /// Charge Full
            {
//                HAL_GPIO_WritePin(LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
            }
        }
//        else
//        {
//            /* Idle: GREEN_LED off, RED_LED off */
//            HAL_GPIO_WritePin(LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
//        }

        HAL_Delay(300);
	}
  /* USER CODE END 3 */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        if(htim->Channel == TIM_CHANNEL_1)
        {
//            BAT_INST = HAL_GPIO_ReadPin(BAT_INST_GPIO_Port, BAT_INST_Pin)^0x01;
            BAT_INST = HAL_GPIO_ReadPin(BAT_INST_GPIO_Port, BAT_INST_Pin)^0x00;
        }
    }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc == &hadc1)
	{
		ADC1_IDLE = 1;
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart == &huart3)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, hostData, 128);

		UART3_RX_DMA_IDLE = 1;
	}
	else if(huart == &huart1)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(huart, rs485Data, 128);
		UART1_RX_DMA_IDLE = 1;
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
	Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
	Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
