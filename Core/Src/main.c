/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEMP_SENSOR_AVG_SLOPE_PER_CELSIUS 2.5f
#define TEMP_SENSOR_VOLTAGE_AT_25 760.0f
#define ADC_REFERENCE_VOLTAGE 3000.0f
#define ADC_MAX_OUTPUT_VALUE 4095.0f
#define BUFF_TX_SIZE 30
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t adc_value, i, count = 0;
uint8_t adc_dma_flag, uart_dma_flag, tick_500ms_elapsed;
float temp;
char usart_tx_dma_buffer[BUFF_TX_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Sensor_Temp_Start(void);
void Usart_TX_Start(void);

float calc_temperature_sensor(uint32_t value);
void uart_transmission(char* buff_tx, float value, int period);
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
	uint32_t return_code;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Sensor_Temp_Start();
  Usart_TX_Start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  return_code = SysTick_Config(SystemCoreClock / 16000);  /* Systick_Config to configure the interrupts to every 1ms */
  if (return_code != 0)
  {
	  Error_Handler();              /* Error Handler */
  }

  while (1)
  {
	  if(tick_500ms_elapsed == 1)
	  {
		  count++;
		  if(adc_dma_flag == 1){
			  temp = calc_temperature_sensor(adc_value);
			  adc_dma_flag = 0;
		  }
		  if(uart_dma_flag == 1){
			  uart_transmission(usart_tx_dma_buffer, temp, (count*500));
			  uart_dma_flag = 0;
		  }
		  tick_500ms_elapsed = 0;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	LL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	LL_mDelay(1000);

	LL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
	LL_mDelay(1000);

	LL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
	LL_mDelay(1000);

	LL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
	LL_mDelay(1000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 168, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(84000000);
  LL_SetSystemCoreClock(84000000);
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Function to process the sensor temperature.
  * @param  value of the ADC's read
  * @retval temp_in_c (Sensor temperature)
  */
float calc_temperature_sensor(uint32_t value)
{
    float temp_aux, temp_in_c;
    temp_aux = ((float)value) / ADC_MAX_OUTPUT_VALUE * ADC_REFERENCE_VOLTAGE;
    temp_in_c = (temp_aux - TEMP_SENSOR_VOLTAGE_AT_25) / TEMP_SENSOR_AVG_SLOPE_PER_CELSIUS + 25;
    return temp_in_c;
}
/**
  * @brief  This function executed the transmission through UART.
  * @param  char  -> buffer array for the transmissión
  * @param  float -> calculated temperature value
  * @param  int   -> period of the timer
  * @retval None
  */
void uart_transmission(char* buff_tx, float value, int period)
{
    Usart_TX_Start();
    sprintf(buff_tx, "%d ms, ADC_T(C):%f \r\n", period, value);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
