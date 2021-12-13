/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GREEN_ON_TIME 	3000
#define YELLOW_ON_TIME 	1000
#define RED_ON_TIME 	3000
#define AT_BUFFER 		20
#define AT_CMGF_BUFFER 	20
#define AT_CNMI_BUFFER	30
#define OK_BUFFER		200
#define MESSAGE_BUFFER	200
#define TRANSMIT_DELAY	3000
#define LEDS_DELAY		3000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for Normal_Traffic */
osThreadId_t Normal_TrafficHandle;
const osThreadAttr_t Normal_Traffic_attributes = {
  .name = "Normal_Traffic",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TrafficLights */
osThreadId_t TrafficLightsHandle;
const osThreadAttr_t TrafficLights_attributes = {
  .name = "TrafficLights",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Override */
osThreadId_t OverrideHandle;
const osThreadAttr_t Override_attributes = {
  .name = "Override",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh6,
};
/* USER CODE BEGIN PV */
char AT[AT_BUFFER];
//int		AT;
char AT_CMGF[AT_CMGF_BUFFER];
//int 	AT_CMGF;
char AT_CNMI[AT_CNMI_BUFFER];
//int 	AT_CNMI;
uint8_t ATisOK = 1;
//uint8_t slot = 0;
uint8_t OK_AT[OK_BUFFER] = { 0 };
uint8_t OK_AT_CMGF[OK_BUFFER] = { 0 };
uint8_t message_1[MESSAGE_BUFFER] = { 0 };
uint8_t message_2[MESSAGE_BUFFER] = { 0 };
//uint8_t rx_index = 0;
//uint8_t rx_data;
int rounds = 0;
int parser = 0;
int mode = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void Answer(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void traffic_lights_on(int rounds)
{
	switch (rounds % 4) {
	case 0:
		HAL_GPIO_WritePin(GPIOB, Red_Pin | Yellow_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Green_Pin, GPIO_PIN_SET);
		HAL_Delay(GREEN_ON_TIME);
		break;
	case 1:
	case 3:
		HAL_GPIO_WritePin(GPIOB, Red_Pin | Green_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Yellow_Pin, GPIO_PIN_SET);
		HAL_Delay(YELLOW_ON_TIME);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOB, Yellow_Pin | Green_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, Red_Pin, GPIO_PIN_SET);
		HAL_Delay(RED_ON_TIME);
		break;
	}
	//rounds++;
}

void AT_SEND_RECEIVE( UART_HandleTypeDef *huart1)
{
	mode = 1;
	snprintf(AT, AT_BUFFER, "AT\r\n");
	HAL_UART_Transmit(&*huart1, (uint8_t*) AT, AT_BUFFER, TRANSMIT_DELAY);
	HAL_Delay(TRANSMIT_DELAY);
	HAL_UART_Receive_IT(&*huart1, OK_AT, OK_BUFFER);
}

void AT_CMGF_SEND_RECEIVE(UART_HandleTypeDef *huart1)
{
	mode = 2;
	snprintf(AT_CMGF, AT_CMGF_BUFFER, "AT+CMGF=1\r\n");
	HAL_UART_Transmit(&*huart1, (uint8_t*) AT_CMGF, AT_CMGF_BUFFER,
			TRANSMIT_DELAY);
	HAL_Delay(2 * TRANSMIT_DELAY);
	HAL_UART_Receive_IT(&*huart1, OK_AT_CMGF, OK_BUFFER);
}

void AT_CNMI_SEND_RECEIVE(UART_HandleTypeDef *huart1)
{
	mode = 3;
	snprintf(AT_CNMI, AT_CNMI_BUFFER, "AT+CNMI=2,2,0,0,0\r\n");
	HAL_UART_Transmit(&*huart1, (uint8_t*) AT_CNMI, MESSAGE_BUFFER,
			TRANSMIT_DELAY);
	HAL_Delay(2 * TRANSMIT_DELAY);
	HAL_UART_Receive_IT(&*huart1, message_1, MESSAGE_BUFFER);
	HAL_UART_Receive_IT(&*huart1, message_2, MESSAGE_BUFFER);
}

void LEDS_ON()
{
	HAL_GPIO_WritePin(GPIOB, Green_Pin | Red_Pin | Yellow_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
}

void LEDS_OFF()
{
	HAL_GPIO_WritePin(GPIOB, Green_Pin | Red_Pin | Yellow_Pin, GPIO_PIN_RESET);
	HAL_Delay(3000);
}

void GREEN_WAVE(int time)
{
	HAL_GPIO_WritePin(GPIOB, Red_Pin | Yellow_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Green_Pin, GPIO_PIN_SET);
	HAL_Delay(time);
}

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Normal_Traffic */
  Normal_TrafficHandle = osThreadNew(StartDefaultTask, NULL, &Normal_Traffic_attributes);

  /* creation of TrafficLights */
  TrafficLightsHandle = osThreadNew(StartTask02, NULL, &TrafficLights_attributes);

  /* creation of Override */
  OverrideHandle = osThreadNew(Answer, NULL, &Override_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Green_Pin|Red_Pin|Yellow_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Flag_GPIO_Port, Flag_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Green_Pin Red_Pin Yellow_Pin */
  GPIO_InitStruct.Pin = Green_Pin|Red_Pin|Yellow_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Flag_Pin */
  GPIO_InitStruct.Pin = Flag_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Flag_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
	  switch(mode)
	  {
	  case 1:
		  if( OK_AT == "OK")
		  {
			  mode = 2;
		  }
	  	break;
	  case 2:
		  if( OK_AT_CMGF == "OK")
		  {
			  mode = 3;
		  }
		  break;
	  case 3:
	  default:
		  if( message_2 == "pass")
		  {
			  mode = 4;
		  }
		  break;
	  }
//    /* Transmit one byte with 100 ms timeout */
//    HAL_UART_Transmit(&huart1, &byte, 1, 100);
//
//    /* Receive one byte in interrupt mode */
//    HAL_UART_Receive_IT(&huart1, &byte, 1);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	traffic_lights_on(rounds);
	rounds++;
	//HAL_UART_Transmit(&huart2, "Test ", 6, 200);
	AT_SEND_RECEIVE(&huart1);
//	HAL_UART_

    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the TrafficLights thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
	  switch(mode)
	  	  {
	  	  case 1:
	  			  AT_CMGF_SEND_RECEIVE(&huart1);
	  			  break;
	  	  case 2:
	  			  AT_CNMI_SEND_RECEIVE(&huart1);
	  			  break;
	  	  case 3:
	  			  AT_CNMI_SEND_RECEIVE(&huart1);
	  			  break;
	  	  case 4:
	  		  	  GREEN_WAVE(5000);
	  		  	  break;
	  	  default:
	  		  	  break;
	  	  }

	  osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_Answer */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Answer */
void Answer(void *argument)
{
  /* USER CODE BEGIN Answer */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Answer */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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

