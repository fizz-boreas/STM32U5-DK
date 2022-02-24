/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"

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
#define GPIO1_Pin GPIO_PIN_3
#define GPIO1_GPIO_Port GPIOE
#define GPIO3_Pin GPIO_PIN_1
#define GPIO3_GPIO_Port GPIOE
#define USER_Pin GPIO_PIN_5
#define USER_GPIO_Port GPIOD
#define GPIO0_Pin GPIO_PIN_4
#define GPIO0_GPIO_Port GPIOE
#define GPIO2_Pin GPIO_PIN_2
#define GPIO2_GPIO_Port GPIOE
#define I2C1_RESET_Pin GPIO_PIN_4
#define I2C1_RESET_GPIO_Port GPIOB
#define D1_g_Pin GPIO_PIN_8
#define D1_g_GPIO_Port GPIOG
#define D1_b_Pin GPIO_PIN_3
#define D1_b_GPIO_Port GPIOG
#define D1_r_Pin GPIO_PIN_5
#define D1_r_GPIO_Port GPIOG
#define D2_g_Pin GPIO_PIN_13
#define D2_g_GPIO_Port GPIOD
#define D2_b_Pin GPIO_PIN_15
#define D2_b_GPIO_Port GPIOD
#define D2_r_Pin GPIO_PIN_12
#define D2_r_GPIO_Port GPIOD
#define HOLD_FLASH_Pin GPIO_PIN_11
#define HOLD_FLASH_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define NBR_OF_PCA9574 2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
