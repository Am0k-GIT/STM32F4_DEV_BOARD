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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
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

#define PULSE_TRIG_US                            10                            // uS
#define DEAD_TIME                                00                            // uS
#define REACT_TIME                               12000                         // uS
#define BLIND_DISTACE_MIN                        1000                          // uS = 0.17 m in air and 0.75 m in water
#define BLIND_DISTACE_MAX                        20000                         // uS = 5.16 m in air and 22.22 m in water
#define SOUND_SPEED                              344                           // 344 m/S in air and 1481 m/S in water 20°С

#define APB1_TIM_FREQ                            42000000                      // TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM12, TIM13, TIM14
#define APB2_TIM_FREQ                            84000000                      // TIM1, TIM8, TIM9, TIM10, TIM11

#define TIM2_COUNT_PER_HZ                        1000000                       // if 1'000'000 one tick = 1uS
#define TIM2_FREQ                                20                            // Hz

#define TIM5_COUNT_PER_HZ                        1000000                       // if 1'000'000 one tick = 1uS
#define TIM5_FREQ                                20                            // Hz

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

char TX_Buffer[64];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void TIMers_Init (void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  sprintf (TX_Buffer, "SR04 test\n");
  HAL_UART_Transmit (&huart1, (uint8_t*) TX_Buffer, strlen (TX_Buffer), 20);

  TIMers_Init ();

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
}

/* USER CODE BEGIN 4 */

void TIMers_Init (void)
{
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  uint32_t tim_clock;

  // Проверяем делитель APB1 (биты PPRE1 в регистре CFGR)
  // Если делитель шины не равен 1 (000), частота таймера = PCLK1 * 2
  if ((RCC->CFGR & RCC_CFGR_PPRE1) == 0)
  {
      tim_clock = pclk1;
  }
  else
  {
      tim_clock = pclk1 * 2;
  }

  TIM2->PSC = (tim_clock / TIM2_COUNT_PER_HZ) - 1;
  TIM2->ARR = (TIM2_COUNT_PER_HZ / TIM2_FREQ) - 1;
  TIM2->EGR |= TIM_EGR_UG;

  TIM2->CCR2 = PULSE_TRIG_US;                                                  // uS

  TIM5->PSC = (tim_clock / TIM5_COUNT_PER_HZ) - 1;
  TIM5->ARR = (TIM5_COUNT_PER_HZ / TIM5_FREQ) - 1;
  TIM5->EGR |= TIM_EGR_UG;

  HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_2);

  HAL_TIM_IC_Start_IT (&htim5, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT (&htim5, TIM_CHANNEL_2);
}

void HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef *htim)                      // колбек по захвату
{
  if (htim->Instance == TIM5)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)                             // RISING с LOW на HIGH
    {
      __HAL_TIM_SET_COUNTER(&htim5, 0x0000);                                   // обнуление счётчика
    }

    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)                        // FALLING с HIGH на LOW
    {
      uint32_t time = HAL_TIM_ReadCapturedValue (&htim5, TIM_CHANNEL_2) - DEAD_TIME;
      // чтение значения в регистре захвата/сравнения

      sprintf (TX_Buffer, "ECHO TIME: %i uS\n", time);
      HAL_UART_Transmit (&huart1, (uint8_t*) TX_Buffer, strlen (TX_Buffer), 20);

      sprintf(TX_Buffer, "DISTANCE: %i cm\n", (time * SOUND_SPEED) / 20000);
      // переводим в см (* 100) / сек (/ 1'000'000) с учетом двойного пути (/ 2)
      HAL_UART_Transmit (&huart1, (uint8_t*) TX_Buffer, strlen (TX_Buffer), 20);
    }
  }
}

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
