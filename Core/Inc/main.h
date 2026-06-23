/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern int fputc(int ch, FILE *f);
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUZZER_Pin GPIO_PIN_4
#define BUZZER_GPIO_Port GPIOA
#define KEY1_Pin GPIO_PIN_5
#define KEY1_GPIO_Port GPIOA
//#define KEY2_Pin GPIO_PIN_5
//#define KEY2_GPIO_Port GPIOA
//#define OLED_I2C_SCL_Pin GPIO_PIN_1
//#define OLED_I2C_SCL_GPIO_Port GPIOB
//#define OLED_I2C_SDA_Pin GPIO_PIN_2
//#define OLED_I2C_SDA_GPIO_Port GPIOB
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define Charge_EN_Pin GPIO_PIN_3
#define Charge_EN_GPIO_Port GPIOB
#define BAT_INST_Pin GPIO_PIN_4
#define BAT_INST_GPIO_Port GPIOB
#define IR_REC_Pin GPIO_PIN_6
#define IR_REC_GPIO_Port GPIOB
#define IR_EM_Pin GPIO_PIN_7
#define IR_EM_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_8
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_9
#define I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

typedef struct
{
    float estimate_temp;
    float estimate_chrg_volt;
    float estimate_chrg_current;
    float estimate_bat_volt;
    float estimate_pre_chrg_volt;
    uint8_t exception;
}env_secure_t;

extern env_secure_t * pEnvSecure;
extern float CHRG_I, CHRG_V, BAT_V;
extern uint8_t BAT_INST, IR_REC;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
