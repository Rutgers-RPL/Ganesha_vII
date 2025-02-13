/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define FLASH_CS_Pin GPIO_PIN_3
#define FLASH_CS_GPIO_Port GPIOE
#define GYR_INT_Pin GPIO_PIN_0
#define GYR_INT_GPIO_Port GPIOA
#define GYR_CS_Pin GPIO_PIN_1
#define GYR_CS_GPIO_Port GPIOA
#define ACC_INT_Pin GPIO_PIN_2
#define ACC_INT_GPIO_Port GPIOA
#define ACC_CS_Pin GPIO_PIN_3
#define ACC_CS_GPIO_Port GPIOA
#define MAG_CS_Pin GPIO_PIN_4
#define MAG_CS_GPIO_Port GPIOA
#define MAG_INT_Pin GPIO_PIN_4
#define MAG_INT_GPIO_Port GPIOC
#define BMP_INT_Pin GPIO_PIN_15
#define BMP_INT_GPIO_Port GPIOE
#define SDMMC1_CD_Pin GPIO_PIN_15
#define SDMMC1_CD_GPIO_Port GPIOA
#define BAT_VOLT_Pin GPIO_PIN_3
#define BAT_VOLT_GPIO_Port GPIOD
#define CAM_FIRE_Pin GPIO_PIN_4
#define CAM_FIRE_GPIO_Port GPIOD
#define CAM_SENSE_Pin GPIO_PIN_5
#define CAM_SENSE_GPIO_Port GPIOD
#define PYRO0_FIRE_Pin GPIO_PIN_6
#define PYRO0_FIRE_GPIO_Port GPIOD
#define PYRO0_SENSE_Pin GPIO_PIN_7
#define PYRO0_SENSE_GPIO_Port GPIOD
#define PYRO1_FIRE_Pin GPIO_PIN_4
#define PYRO1_FIRE_GPIO_Port GPIOB
#define PYRO1_SENSE_Pin GPIO_PIN_5
#define PYRO1_SENSE_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_8
#define LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
