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
#include "fatfs.h"
#include "sdio.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void SDIO_SDCard_Test(void);
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
  MX_SDIO_SD_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  SDIO_SDCard_Test();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
static void SDIO_SDCard_Test(void)
{
  FATFS FatFs;
  FIL Fil;
  FRESULT FR_Status;
  FATFS *FS_Ptr;
  UINT RWC, WWC; // Read/Write Word Counter
  DWORD FreeClusters;
  uint32_t TotalSize, FreeSpace;
  char RW_Buffer[256];
  char Tx_Buffer[300];

  printf(Tx_Buffer, "TEST SDIO interface.\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)Tx_Buffer, strlen(Tx_Buffer), 20);

  do
  {
    //------------------[ Mount The SD Card ]--------------------
    FR_Status = f_mount(&FatFs, SDPath, 1);
    if (FR_Status != FR_OK)
    {
        sprintf(Tx_Buffer, "Error! While Mounting SD Card, Error Code: (%i)\r\n", FR_Status);
        HAL_UART_Transmit(&huart1, (uint8_t*)Tx_Buffer, strlen(Tx_Buffer), 20);
        break;
    }
    else
    {
        sprintf(Tx_Buffer, "SD Card Mounted Successfully! \r\n\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)Tx_Buffer, strlen(Tx_Buffer), 20);
        if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
        {
            sprintf(Tx_Buffer, "Cant shange SDIO to 4bit.\r\n\n");
            HAL_UART_Transmit(&huart1, (uint8_t*)Tx_Buffer, strlen(Tx_Buffer), 20);
        }
        else
        {
            sprintf(Tx_Buffer, "Change SDIO to 4bit mode success!\r\n\n");
            HAL_UART_Transmit(&huart1, (uint8_t*)Tx_Buffer, strlen(Tx_Buffer), 20);
        }
    }

    //------------------[ Get & Print The SD Card Size & Free Space ]--------------------
    f_getfree("", &FreeClusters, &FS_Ptr);
    TotalSize = (uint32_t)((FS_Ptr->n_fatent - 2) * FS_Ptr->csize * 0.5);
    FreeSpace = (uint32_t)(FreeClusters * FS_Ptr->csize * 0.5);
    sprintf(Tx_Buffer, "Total SD Card Size: %lu KBytes\r\n", TotalSize);
    HAL_UART_Transmit(&huart1, (uint8_t*)Tx_Buffer, strlen(Tx_Buffer), 20);
    sprintf(Tx_Buffer, "Free SD Card Space: %lu KBytes\r\n\n", FreeSpace);
    HAL_UART_Transmit(&huart1, (uint8_t*)Tx_Buffer, strlen(Tx_Buffer), 20);
    //------------------[ Open A Text File For Write & Write Data ]--------------------
    //Open the file
    FR_Status = f_open(&Fil, "log.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
    if(FR_Status != FR_OK)
    {
      sprintf(Tx_Buffer, "Error! While Creating/Opening A New Text File, Error Code: (%i)\r\n", FR_Status);
      HAL_UART_Transmit(&huart1, (uint8_t*)Tx_Buffer, strlen(Tx_Buffer), 20);
      break;
    }
    sprintf(Tx_Buffer, "Text File Created & Opened! Writing Data To The Text File..\r\n\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)Tx_Buffer, strlen(Tx_Buffer), 20);
    // (1) Write Data To The Text File [ Using f_puts() Function ]
    f_puts("Hello! From STM32 To SD Card Over SDIO, Using f_puts()\n", &Fil);
    // (2) Write Data To The Text File [ Using f_write() Function ]
    strcpy(RW_Buffer, "Hello! From STM32 To SD Card Over SDIO, Using f_write()\r\n");
    f_write(&Fil, RW_Buffer, strlen(RW_Buffer), &WWC);
    // Close The File
    f_close(&Fil);
    //------------------[ Open A Text File For Read & Read Its Data ]--------------------
    // Open The File
    FR_Status = f_open(&Fil, "log.txt", FA_READ);
    if(FR_Status != FR_OK)
    {
      sprintf(Tx_Buffer, "Error! While Opening (log.txt) File For Read.. \r\n");
      HAL_UART_Transmit(&huart1, (uint8_t*)Tx_Buffer, strlen(Tx_Buffer), 20);
      break;
    }
    // (1) Read The Text File's Data [ Using f_gets() Function ]
    f_gets(RW_Buffer, sizeof(RW_Buffer), &Fil);
    sprintf(Tx_Buffer, "Data Read From (log.txt) Using f_gets(): %s", RW_Buffer);
    HAL_UART_Transmit(&huart1, (uint8_t*)Tx_Buffer, strlen(Tx_Buffer), 20);
    // (2) Read The Text File's Data [ Using f_read() Function ]
    f_read(&Fil, RW_Buffer, f_size(&Fil), &RWC);
    sprintf(Tx_Buffer, "Data Read From (log.txt) Using f_read(): %s", RW_Buffer);
    HAL_UART_Transmit(&huart1, (uint8_t*)Tx_Buffer, strlen(Tx_Buffer), 20);
    // Close The File
    f_close(&Fil);
    sprintf(Tx_Buffer, "File Closed! \r\n\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)Tx_Buffer, strlen(Tx_Buffer), 20);
    //------------------[ Open An Existing Text File, Update Its Content, Read It Back ]--------------------
    // (1) Open The Existing File For Write (Update)
    FR_Status = f_open(&Fil, "log.txt", FA_OPEN_EXISTING | FA_WRITE);
    FR_Status = f_lseek(&Fil, f_size(&Fil)); // Move The File Pointer To The EOF (End-Of-File)
    if(FR_Status != FR_OK)
    {
      sprintf(Tx_Buffer, "Error! While Opening (log.txt) File For Update.. \r\n");
      HAL_UART_Transmit(&huart1, (uint8_t*)Tx_Buffer, strlen(Tx_Buffer), 20);
      break;
    }
    // (2) Write New Line of Text Data To The File
    FR_Status = f_puts("This New Line Was Added During File Update!\r\n\n", &Fil);
    f_close(&Fil);
    memset(RW_Buffer,'\0',sizeof(RW_Buffer)); // Clear The Buffer
    // (3) Read The Contents of The Text File After The Update
    FR_Status = f_open(&Fil, "log.txt", FA_READ); // Open The File For Read
    f_read(&Fil, RW_Buffer, f_size(&Fil), &RWC);
    sprintf(Tx_Buffer, "Data Read From (log.txt) After Update:\r\n%s", RW_Buffer);
    HAL_UART_Transmit(&huart1, (uint8_t*)Tx_Buffer, strlen(Tx_Buffer), 50);
    f_close(&Fil);
    //------------------[ Delete The Text File ]--------------------
    // Delete The File
    //FR_Status = f_unlink(MyTextFile.txt);
    //if (FR_Status != FR_OK){
     //   sprintf(Tx_Buffer, "Error! While Deleting The (MyTextFile.txt) File.. \r\n");
      //  HAL_UART_Transmit(&huart1, (uint8_t*)Tx_Buffer, strlen(Tx_Buffer), 20);
    //}
  } while(0);
  //------------------[ Test Complete! Unmount The SD Card ]--------------------
  FR_Status = f_mount(NULL, "", 0);
  if (FR_Status != FR_OK)
  {
      sprintf(Tx_Buffer, "\r\nError! While Un-mounting SD Card, Error Code: (%i)\r\n", FR_Status);
      HAL_UART_Transmit(&huart1, (uint8_t*)Tx_Buffer, strlen(Tx_Buffer), 20);
  }
  else
  {
      sprintf(Tx_Buffer, "\r\nSD Card Un-mounted Successfully! \r\n");
      HAL_UART_Transmit(&huart1, (uint8_t*)Tx_Buffer, strlen(Tx_Buffer), 20);

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
