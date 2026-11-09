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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include "stm32746g_discovery_lcd.h"
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
uint8_t rx_byte;
volatile uint8_t rb[512];
volatile uint16_t rb_head = 0 , rb_tail = 0;
volatile uint8_t  last_byte = 0;
volatile uint32_t rx_count = 0;

char line_buf[64];
volatile uint16_t line_len = 0;
volatile uint32_t line_ok =0, line_err = 0;
volatile uint32_t  last_raw = 0;
volatile uint8_t last_kind =0 ;

volatile  uint8_t lcd_ret = 0xff;
//uint32_t t_text = 0;

volatile uint32_t last_ch1 = 0;
volatile uint32_t last_ch2 = 0;
volatile uint8_t last_dig = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
   MPU_Region_InitTypeDef mpu = {0};

  HAL_MPU_Disable();

  mpu.Enable           = MPU_REGION_ENABLE;
  mpu.Number           = MPU_REGION_NUMBER1;
  mpu.BaseAddress      = 0xC0000000;
  mpu.Size             = MPU_REGION_SIZE_32MB;
  mpu.SubRegionDisable = 0x00;
  mpu.TypeExtField     = MPU_TEX_LEVEL0;
  mpu.AccessPermission = MPU_REGION_FULL_ACCESS;
  mpu.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
  mpu.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  mpu.IsCacheable      = MPU_ACCESS_CACHEABLE;
  mpu.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&mpu);

  mpu.Number           = MPU_REGION_NUMBER2;
  mpu.BaseAddress      = 0xA0000000;
  mpu.Size             = MPU_REGION_SIZE_8KB;
  mpu.SubRegionDisable = 0x00;
  mpu.TypeExtField     = MPU_TEX_LEVEL0;
  mpu.AccessPermission = MPU_REGION_FULL_ACCESS;
  mpu.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  mpu.IsShareable      = MPU_ACCESS_SHAREABLE;
  mpu.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  mpu.IsBufferable     = MPU_ACCESS_BUFFERABLE;
  HAL_MPU_ConfigRegion(&mpu);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

  lcd_ret = BSP_LCD_Init();

  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
  BSP_LCD_DisplayOn();
  BSP_LCD_SelectLayer(0);
  BSP_LCD_Clear(LCD_COLOR_BLACK);

  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_DisplayStringAt(8, 8, (uint8_t *)"LCD OK", LEFT_MODE);

  BSP_LCD_SetTextColor(LCD_COLOR_RED);
  BSP_LCD_DrawLine(0, 136, 479, 136);

  HAL_UART_Receive_IT(&huart6, &rx_byte, 1);
HAL_UART_Receive_IT(&huart6,&rx_byte , 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    
//    if(HAL_GetTick() - t_text >= 1000)
//    {
//      t_text = HAL_GetTick();
//      uint8_t msg[] = "A1,2048\r\n";
//      HAL_UART_Transmit(&huart6, msg,sizeof (msg) - 1,20);
//    }
//    if(HAL_GetTick() - t_text >= 3000)
//    {
//      t_text = HAL_GetTick();
//      static uint8_t test_step = 0;
//      uint8_t m0[] = "A2,100,200,1\r\n";
//      uint8_t m1[] = "A2,300,400,2\r\n";
//      uint8_t m2[] = "A2,500,600,4\r\n";
//      
//      if(test_step == 0)
//      {
//        HAL_UART_Transmit(&huart6, m0,sizeof (m0) - 1,20);
//      }
//      else if(test_step == 1)
//      {
//        HAL_UART_Transmit(&huart6, m1,sizeof (m1) - 1,20);
//      }
//      else
//      {
//         HAL_UART_Transmit(&huart6,m2,sizeof (m2) - 1,20);
//      }
//      
//      test_step++;
//      if(test_step >= 3)
//      {
//        test_step =0;
//      }
//    }
    
    
    while (rb_tail != rb_head)
    {
      uint8_t b  = rb[rb_tail];
      rb_tail = (uint16_t)((rb_tail + 1) & 511);
      rx_count++;
      
      if(b == '\n')
      {
        unsigned int a = 0,b =0, c =0 ;
        line_buf[line_len] = '\0';
        
        if(line_len > 0 && line_buf[line_len-1]=='\r')
        {
          line_buf [line_len -1] = '\0';
        }
        if(sscanf(line_buf, "A2,%u,%u,%u",&a,&b,&c)== 3)
        {
          last_ch1 =a;
          last_ch2 =b;
          last_dig = (uint8_t)c;
          last_kind = '2';
          line_ok++;
        }
        else if(sscanf(line_buf,"A1,%u",&a)== 1)
          {
            last_raw =a;
            last_kind = '1';
            line_ok++;
          }
        
        else
        {
          line_err++;
        }
        line_len = 0;
        
      }
      else
      {
        if(line_len < 63)
        {
          line_buf[line_len] = (char)b;
          line_len++;
        }
        else
        {
          line_len = 0;
        }
      }
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
  uint16_t next = (uint16_t)((rb_head + 1) & 511);
  if(next != rb_tail)
  {
    rb[rb_head] = rx_byte;
    rb_head = next;
  }
  HAL_UART_Receive_IT(&huart6, &rx_byte,1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if( huart->Instance == USART6)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_Receive_IT(&huart6, &rx_byte,1);
  }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
