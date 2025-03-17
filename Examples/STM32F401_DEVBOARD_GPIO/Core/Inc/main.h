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
#include "stm32f4xx_hal.h"

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
#define L_2_Pin GPIO_PIN_0
#define L_2_GPIO_Port GPIOB
#define L_3_Pin GPIO_PIN_1
#define L_3_GPIO_Port GPIOB
#define B_3_Pin GPIO_PIN_12
#define B_3_GPIO_Port GPIOB
#define B_3_EXTI_IRQn EXTI15_10_IRQn
#define B_2_Pin GPIO_PIN_13
#define B_2_GPIO_Port GPIOB
#define B_2_EXTI_IRQn EXTI15_10_IRQn
#define B_1_Pin GPIO_PIN_14
#define B_1_GPIO_Port GPIOB
#define B_1_EXTI_IRQn EXTI15_10_IRQn
#define B_0_Pin GPIO_PIN_15
#define B_0_GPIO_Port GPIOB
#define B_0_EXTI_IRQn EXTI15_10_IRQn
#define L_0_Pin GPIO_PIN_6
#define L_0_GPIO_Port GPIOC
#define L_1_Pin GPIO_PIN_7
#define L_1_GPIO_Port GPIOC
#define SDIO_CD_Pin GPIO_PIN_15
#define SDIO_CD_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
