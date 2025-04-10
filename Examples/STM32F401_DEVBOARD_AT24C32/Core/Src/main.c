/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "24CXX/ee24.h"
#include "BUTTONS/buttons.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define EEPROM_SIZE                      4096
#define CTRL_BYTE                  0b01010101

#define DATA_START_ADDRESS                 32
#define DATA_STOP_ADDRESS                  63
#define DATA_STRUCT_START_ADDRESS         256
#define DATA_STRUCT_STOP_ADDRESS          512

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

char data_str[64] = {0,};
EE24_HandleTypeDef ee24;
uint8_t dataRead_Full_ee24[EEPROM_SIZE];

bool flag_erase = false;
bool flag_show = false;
bool flag_read = false;
bool flag_write = false;
bool flag_read_struct = false;
bool flag_write_struct = false;

uint8_t DataToWrite[5] = {11, 12, 13, 14, 15};
uint8_t DataToRead[5] =  { 0,  0,  0,  0,  0};

typedef struct
{
    uint16_t raw_value;
    uint16_t temp_value;
} table_entry_t;

typedef struct
{
        uint16_t P;
        uint16_t I;
        uint16_t D;
        table_entry_t table_array[4];
} struct_entry_t;

table_entry_t TABLE_EXAMPLE[4] =
{
        {0, 0},
        {100, 200},
        {300, 900},
        {600, 4000},
};

struct_entry_t STRUCT_ARRAY[3] = {};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Button_React(uint16_t key_number, bool longpress);
bool EEPROM_Init (void);
void EEPROM_Erase (void);
void EEPROM_Show (void);

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  snprintf(data_str, 63, "AT24C32 DRIVER EXAMPLE.\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)data_str, strlen(data_str), 10);

  snprintf(data_str, 63, "B_3 PRESS - show EEPROM\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)data_str, strlen(data_str), 10);

  snprintf(data_str, 63, "B_3 LONG PRESS - full erase EEPROM\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)data_str, strlen(data_str), 10);

  snprintf(data_str, 63, "B_2 PRESS - read data array from EEPROM\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)data_str, strlen(data_str), 10);

  snprintf(data_str, 63, "B_2 LONG PRESS - read struct array from EEPROM\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)data_str, strlen(data_str), 10);

  snprintf(data_str, 63, "B_1 PRESS - write data array from EEPROM\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)data_str, strlen(data_str), 10);

  snprintf(data_str, 63, "B_1 LONG PRESS - write struct array from EEPROM\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)data_str, strlen(data_str), 10);

  if (EEPROM_Init())
  {
        snprintf(data_str, 63, "EEPROM INIT\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)data_str, strlen(data_str), 10);
        EEPROM_Show ();
  }
  else
  {
        snprintf(data_str, 63, "EEPROM NOT INIT\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)data_str, strlen(data_str), 10);
  }

  // Инициация массива структур для записи
  for (uint8_t i = 0; i < 3; i++)
  {
    STRUCT_ARRAY[i].P = i + 10;
    STRUCT_ARRAY[i].I = i + 20;
    STRUCT_ARRAY[i].D = i + 30;
    for (uint8_t k = 0; k < 4; k++)
    {
      STRUCT_ARRAY[i].table_array[k] = TABLE_EXAMPLE[k];
    }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    BUTTON_IRQ_Debounce(Button_React);

    if (flag_show)
    {
      EEPROM_Show ();
      flag_show = false;
    }

    if (flag_erase)
    {
      snprintf (data_str, 63, "FULL ERASE, PLEASE WAIT\n");
      HAL_UART_Transmit (&huart1, (uint8_t*) data_str, strlen (data_str), 10);
      EEPROM_Erase ();
      snprintf (data_str, 63, "ERASE COMPLETE\n");
      HAL_UART_Transmit (&huart1, (uint8_t*) data_str, strlen (data_str), 10);
      flag_erase = false;
    }

    if (flag_read)
    {
      snprintf (data_str, 63, "READ CIRCLE EEPROM\n");
      HAL_UART_Transmit (&huart1, (uint8_t*) data_str, strlen (data_str), 10);
      EE24_ReadCircle (&ee24, DATA_START_ADDRESS, DATA_STOP_ADDRESS, CTRL_BYTE, &DataToRead, sizeof(DataToRead), 10);
      snprintf (data_str, 63, "READ COMPLETE: ");
      HAL_UART_Transmit (&huart1, (uint8_t*) data_str, strlen (data_str), 10);
      for (uint8_t i = 0; i < sizeof(DataToRead); i++)
      {
        snprintf (data_str, 63, " %4d ", DataToRead[i]);
        HAL_UART_Transmit (&huart1, (uint8_t*) data_str, strlen (data_str), 10);
      }
      snprintf (data_str, 63, "\n");
      HAL_UART_Transmit (&huart1, (uint8_t*) data_str, strlen (data_str), 10);
      flag_read = false;
    }

    if (flag_write)
    {
      snprintf (data_str, 63, "WRITE CIRCLE EEPROM\n");
      HAL_UART_Transmit (&huart1, (uint8_t*) data_str, strlen (data_str), 10);
      EE24_WriteCircle (&ee24, DATA_START_ADDRESS, DATA_STOP_ADDRESS, CTRL_BYTE, &DataToWrite, sizeof(DataToWrite), 10);
      snprintf (data_str, 63, "WRITE COMPLETE\n");
      HAL_UART_Transmit (&huart1, (uint8_t*) data_str, strlen (data_str), 10);
      flag_write = false;
    }

    if (flag_read_struct)
    {
      struct_entry_t new_STRUCT_ARRAY[3] = { };
      snprintf (data_str, 63, "READ STRUCT SIZE: %4d \n", sizeof(new_STRUCT_ARRAY));
      HAL_UART_Transmit (&huart1, (uint8_t*) data_str, strlen (data_str), 10);

      EE24_ReadCircle (&ee24, DATA_STRUCT_START_ADDRESS, DATA_STRUCT_STOP_ADDRESS, CTRL_BYTE, &new_STRUCT_ARRAY, sizeof(new_STRUCT_ARRAY), 10);

      for (uint8_t i = 0; i < 3; i++)
      {
        snprintf (data_str, 63, "STRUCT[%1d]: %4d %4d %4d ", i, new_STRUCT_ARRAY[i].P, new_STRUCT_ARRAY[i].I, new_STRUCT_ARRAY[i].D);
        HAL_UART_Transmit (&huart1, (uint8_t*) data_str, strlen (data_str), 10);
        for (uint8_t k = 0; k < 4; k++)
        {
          snprintf (data_str, 63, " [ %4d %4d ] ", new_STRUCT_ARRAY[i].table_array[k].raw_value,
                    new_STRUCT_ARRAY[i].table_array[k].temp_value);
          HAL_UART_Transmit (&huart1, (uint8_t*) data_str, strlen (data_str), 10);
        }
        snprintf (data_str, 63, "\n");
        HAL_UART_Transmit (&huart1, (uint8_t*) data_str, strlen (data_str), 10);
      }
      flag_read_struct = false;
    }

    if (flag_write_struct)
    {
      snprintf (data_str, 63, "WRITE STRUCT SIZE: %4d \n", sizeof(STRUCT_ARRAY));
      HAL_UART_Transmit (&huart1, (uint8_t*) data_str, strlen (data_str), 10);

      EE24_WriteCircle (&ee24, DATA_STRUCT_START_ADDRESS, DATA_STRUCT_STOP_ADDRESS, CTRL_BYTE, &STRUCT_ARRAY, sizeof(STRUCT_ARRAY), 10);

      snprintf (data_str, 63, "WRITE COMPLETE\n");
      HAL_UART_Transmit (&huart1, (uint8_t*) data_str, strlen (data_str), 10);
      flag_write_struct = false;
    }
    HAL_Delay(10);
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

bool EEPROM_Init (void)
{
    if (EE24_Init(&ee24, &hi2c1, EE24_ADDRESS_DEFAULT))
        return 1;
    else
        return 0;
}

void EEPROM_Erase (void)
{
    uint8_t empty_byte = 0xFF;
    for (uint16_t  i = 0; i < EEPROM_SIZE ; i++)
    {
        EE24_Write(&ee24, i, &empty_byte, 1, 10);
    }
}

void EEPROM_Show (void)
{
    EE24_Read(&ee24, 0u, dataRead_Full_ee24, EEPROM_SIZE, 1000);
    HAL_Delay(10);

    snprintf(data_str, 63, "\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)data_str, strlen(data_str), 10);

    snprintf(data_str, 63, "  EEPROM  I  ");
    HAL_UART_Transmit(&huart1, (uint8_t*)data_str, strlen(data_str), 10);

    for (uint8_t j = 0; j < 32; j++)
    {
        snprintf (data_str, 63, " %03d ", j);
        HAL_UART_Transmit (&huart1, (uint8_t*) data_str, strlen (data_str), 10);
    }
    snprintf(data_str, 63, "\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)data_str, strlen(data_str), 10);

    snprintf(data_str, 63, "_____________");
    HAL_UART_Transmit(&huart1, (uint8_t*)data_str, strlen(data_str), 10);

    for (uint8_t j = 0; j < 32; j++)
    {
        snprintf (data_str, 63, "_____", j);
        HAL_UART_Transmit (&huart1, (uint8_t*) data_str, strlen (data_str), 10);
    }
    snprintf(data_str, 63, "\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)data_str, strlen(data_str), 10);

    for (uint8_t i = 0; i < EEPROM_SIZE / 32; i++)
    {
        snprintf (data_str, 63, "%04d-%04d I  ", 32 * i, 32 * i + 31);
        HAL_UART_Transmit (&huart1, (uint8_t*) data_str, strlen (data_str), 10);

        for (uint8_t j = 0; j < 32; j++)
        {
            snprintf (data_str, 63, "0x%02X ", (uint8_t) dataRead_Full_ee24[32 * i + j]);
            HAL_UART_Transmit (&huart1, (uint8_t*) data_str, strlen (data_str), 10);
        }
        snprintf(data_str, 63, "\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)data_str, strlen(data_str), 10);
    }
    snprintf(data_str, 63, "\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)data_str, strlen(data_str), 10);
}

void Button_React(uint16_t key_number, bool longpress)
{
    switch (key_number)
    {
        case (B_3_Pin):
            if (longpress)
              flag_erase = true;
            else
              flag_show = true;
            break;
        case (B_2_Pin):
            if (longpress)
              flag_read_struct = true;
            else
              flag_read = true;
            break;
        case (B_1_Pin):
            if (longpress)
              flag_write_struct = true;
            else
              flag_write = true;
            break;
        case (B_0_Pin):

            break;
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
