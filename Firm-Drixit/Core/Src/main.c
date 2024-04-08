/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "strings.h"

#include "LIS3MDL.h"
#include "W25Q80.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	int32_t ID;
	int16_t axis_X;
	int16_t axis_Y;
	int16_t axis_Z;
	int16_t temp;
} __attribute__((aligned(16))) DATA_STORED; //12 Adding 4bytes to align it to 16(32) to make faster operations

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DATA_SIZE_STORAGE 16 //BYTES
#define FLASH_MAX_BYTES 1048576
#define FLASH_MAX_DATA (FLASH_MAX_BYTES -256 -DATA_SIZE_STORAGE)/DATA_SIZE_STORAGE //Minus the first page(256pytes)- data size
// We´ll reserve the first page for Address index and other possible records (firmware version-features)
// This will leave 65,280 entries of data (ID+AXIS XYZ + TEMP)
#define FLASH_BASE_DATA 0x100
#define FLASH_CLEAR 0x10000 //After the 65,280 bytes of data length that the index can take
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
QueueHandle_t queue_read_ID;
SemaphoreHandle_t sem_get_data;

uint8_t recv_uart;
DATA_STORED Data2Write; //STORE THE LAST DATA FROM THE SENSOR
const uint32_t Index_Add = 0; //First entry for memory ADDRES Index

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
uint32_t GET_addres_from_id(uint32_t ID);
uint32_t GET_id_from_addres(uint32_t ADD);

static void tsk_Store_Data(void *pvParameters);
static void tsk_Flash_Read(void *pvParameters);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*Functions
 *
 */
uint32_t GET_addres_from_id(uint32_t ID){
	return FLASH_BASE_DATA + (ID-1)*DATA_SIZE_STORAGE;
}
uint32_t GET_id_from_addres(uint32_t ADD){
	return ((ADD>>8 -1)*16 + ( ADD & 0xFF )/16 +1);
}
/*Tasks
 *
 */
static void tsk_Store_Data (void *pvParameters){
	BaseType_t _ret = pdFALSE;
	LIS3_DATA DataSensored;

	uint32_t ADD_Last_Entry; //Address of last entry

	W25Q_Read_data((uint8_t *)&ADD_Last_Entry, Index_Add, 4); //4BYTES

	//If the flash memory has been erased and not properly initialized
	if (0xFFFFFFFF == ADD_Last_Entry){
		ADD_Last_Entry = FLASH_BASE_DATA; //It was erased but the index wasn't reseted
	}

	while (1){
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);//test
		xSemaphoreTake(sem_get_data, portMAX_DELAY); //Unblock each 1s
		DataSensored = LIS3MDL_Get_XYZT();

		if( FLASH_MAX_DATA > (ADD_Last_Entry + DATA_SIZE_STORAGE) ){
			taskENTER_CRITICAL();
			ADD_Last_Entry += DATA_SIZE_STORAGE;//Add 16 to address

			Data2Write.axis_X = DataSensored.axis_X;
			Data2Write.axis_Y = DataSensored.axis_Y;
			Data2Write.axis_Z = DataSensored.axis_Z;
			Data2Write.temp   = DataSensored.temp;
/*							|| page #1 ->256 -> (1-1)*16 =0 || Offset 0x0000FF/16 EACH ENTRY	*/
//			Data2Write.ID     = (ADD_Last_Entry>>8 -1)*16 + ( ADD_Last_Entry&0xFF )/16 +1;
			Data2Write.ID     = GET_id_from_addres(ADD_Last_Entry);

			W25Q_Write_data((uint8_t *)&Data2Write, ADD_Last_Entry, DATA_SIZE_STORAGE);
			W25Q_Write_data((uint8_t *)&ADD_Last_Entry, Index_Add , 4); //Store the address of last entry into the Index

			taskEXIT_CRITICAL();
		}
		else{
			//TODO: MSG "NO GRABO"
			//TODO: RESET ID & Address of last entry?
		}
		UNUSED(_ret);
	}
	vTaskDelete( NULL ); //SAFETY
}

static void tsk_Flash_Read (void *pvParameters){
	HAL_StatusTypeDef ret_state;
	BaseType_t _ret = pdFALSE;
	uint32_t read_ID;
	DATA_STORED DATA;

	while (1){
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);//test
		xQueueReceive(queue_read_ID, &read_ID, HAL_MAX_DELAY);

		taskENTER_CRITICAL();
		ret_state = W25Q_Read_data((uint8_t *)&DATA, GET_addres_from_id(read_ID), DATA_SIZE_STORAGE);
		//TODO: Check if the ID that was retrieved is the same as the requested
		ret_state = HAL_UART_Transmit(&huart1, (uint8_t *)&DATA, 12, HAL_MAX_DELAY);
		taskEXIT_CRITICAL();

		UNUSED(ret_state);
		UNUSED(_ret);
	}
	vTaskDelete( NULL ); //SAFETY
}

//UART INTERRUPT
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	BaseType_t pxHigherPriorityTaskWoken = pdFALSE, ret_queue;

	if( USART1 == huart->Instance){
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);//test

		if ( (0 < recv_uart) && (recv_uart <= Data2Write.ID) && ( recv_uart < FLASH_MAX_DATA) ){
			ret_queue = xQueueSendFromISR(queue_read_ID, &recv_uart, &pxHigherPriorityTaskWoken);
			portEND_SWITCHING_ISR(pxHigherPriorityTaskWoken);
			UNUSED(ret_queue);
		}
		else{
			if( recv_uart < FLASH_MAX_DATA){ //Still in the valid range of index
				char msg[] = "ERROR: ID NOT FOUND\n";
				HAL_UART_Transmit(&huart1, (uint8_t)&msg, 20, HAL_MAX_DELAY);
			}
// Clear instructions from UART
			if(recv_uart == FLASH_CLEAR){
				W25Q80_Full_Erase();
				W25Q_Write_data(&FLASH_BASE_DATA, ADD_Index, 4);
			}
			else{
				char msg[] = "ERROR\n";
				HAL_UART_Transmit(&huart1, (uint8_t)&msg, 20, HAL_MAX_DELAY);

			}
		}
	}
}
// BUTON INTERRUPT
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	HAL_StatusTypeDef ret_state;

	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);//test
	//We send immediately the last data from the sensor instead of reading the flash memory
	ret_state = HAL_UART_Transmit_IT(&huart1, (uint8_t *)&Data2Write, 12);//We will send 12 bytes not the 16 aligned
	UNUSED(ret_state);
}
// TIMER INTERRUPT
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

	if(htim->Instance == TIM1){
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);//test

		xSemaphoreGiveFromISR(sem_get_data, &pxHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(pxHigherPriorityTaskWoken);
	}
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

  queue_read_ID = xQueueCreate(1, sizeof(recv_uart));

  vSemaphoreCreateBinary(sem_get_data);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  //Init sensor
  LIS3MDL_init(&hspi1, CS_GIROS_GPIO_Port, CS_GIROS_Pin);
  LIS3MDL_config();
  //Init flash mem
  W25Q_Init(&hspi1,CS_GIROS_GPIO_Port, CS_GIROS_Pin);
  //Int.
  HAL_StatusTypeDef ret_state;
  BaseType_t _ret = pdFALSE;

  ret_state = HAL_TIM_Base_Start_IT(&htim1);//TIMER1 Interrupt call
  ret_state = HAL_UART_Receive_IT(&huart1, &recv_uart, 4); //UART1 Interrupt call - 4bytes
//TODO: check the interrupts starts
  UNUSED(ret_state);

  _ret = xTaskCreate(tsk_Store_Data,
						"",
						configMINIMAL_STACK_SIZE,
						NULL,
						tskIDLE_PRIORITY + 2,
						NULL);

  if (pdFALSE == _ret){
//	  TODO: error
  }
  _ret = xTaskCreate(tsk_Flash_Read,
						"",
						configMINIMAL_STACK_SIZE,
						NULL,
						tskIDLE_PRIORITY + 1,
						NULL);

  if (pdFALSE == _ret){
//	  TODO: error
  }

  vTaskStartScheduler();
  /* USER CODE END 2 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7200-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_GIROS_Pin|CS_FLASH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_GIROS_Pin CS_FLASH_Pin */
  GPIO_InitStruct.Pin = CS_GIROS_Pin|CS_FLASH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

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
