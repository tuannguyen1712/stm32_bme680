/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include "BME68x/bme68x.h"
#include "BME68x/common.h"
#include "SSD1306/menu.h"
#include "SSD1306/ssd1306.h"
#include "SSD1306/ssd1306_conf.h"
#include "SSD1306/ssd1306_fonts.h"
#include "SSD1306/ssd1306_tests.h"
#include "W25Q32/W25Q32.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
int acc_x, acc_y, acc_z, gyz_x, gyz_y, gyz_z;
float tem, gas;

uint8_t uart_recv = 0;
uint8_t uart_chr;
uint8_t uart_buf[100];
uint8_t tx[1024];
uint8_t uart_buf_cnt = 0;
uint8_t uart_chr;
uint8_t uart_last_rcv = 0;

uint8_t uart_data = 1;
uint8_t w25qxx_data = 1;

uint8_t w25qxx_str[1024];

uint8_t w25qxx_output_str[1024];

SemaphoreHandle_t mutex1;
QueueHandle_t uart_queue;
QueueHandle_t w25qxx_queue;

bool rx_spi_flg = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void uart_task();
void w25qxx_task();
void ssd1306_task();
void clear_uart_buf();

void extract_data(const uint8_t *input, int *acc_x, int *acc_y, int *acc_z,
					int *gyz_x, int *gyz_y, int *gyz_z, float *tem, float *gas);
void compress_data(const int acc_x, const int acc_y, const int acc_z,
		const int gyz_x, const int gyz_y, const int gyz_z, const float tem, const float gas);
void Handle_Command();
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void response(char *rsp);
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
  MX_TIM3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	  ssd1306_Init(&hi2c1);
	  W25Q32_Init(&hspi1, GPIOA, GPIO_PIN_3);
//	  sssd1306_temgas(&hi2c1, 1203, 2230, 312, 1243, 2342, 182, 30.023, 980.322);
	  HAL_UART_Receive_IT(&huart1, &uart_chr, sizeof(uart_chr));
	  ssd1306_wait(&hi2c1);
  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  	uart_queue = xQueueCreate(1, sizeof(uint8_t));
  	w25qxx_queue = xQueueCreate(1, sizeof(uint8_t));
  	mutex1 = xSemaphoreCreateMutex();
    xTaskCreate(uart_task, "UART task", 2048, NULL, 2, NULL);
	xTaskCreate(w25qxx_task, "W25QXX task", 2048, NULL, 1, NULL);
	xTaskCreate(ssd1306_task, "SSD1306 task", 2048, NULL, 1, NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	while (0) {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void uart_task() {
	for (;;) {
		Handle_Command();
		if (uart_recv) {
			if (xSemaphoreTake(mutex1, portMAX_DELAY) == pdTRUE) {
				response("UART have mutex\n");
				xQueueSend(uart_queue, (void*) &uart_data, (TickType_t) 0);
				uart_recv = 0;
				xSemaphoreGive(mutex1);
				clear_uart_buf();
				response("UART release mutex\n");
			}
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void w25qxx_task() {
	for (;;) {
		if (uxQueueMessagesWaiting(uart_queue)) {
			if (xSemaphoreTake(mutex1, portMAX_DELAY) == pdTRUE) {
				response("W25Qxx have mutex\n");
				compress_data(acc_x, acc_y, acc_z, gyz_x, gyz_y, gyz_z, tem, gas);
				W25Q32_erase4k(10 * SECTOR_SIZE);
				W25Q32_WriteData(w25qxx_str, 10 * SECTOR_SIZE, strlen((char*) w25qxx_str));
				w25qxx_data = strlen((char*) w25qxx_str);
				xQueueSend(w25qxx_queue, (void*) &w25qxx_data, (TickType_t) 0);
				ssd1306_info(&hi2c1, acc_x, acc_y, acc_z, gyz_x, gyz_y, gyz_z, tem, gas);
				xSemaphoreGive(mutex1);
				response("W25Qxx release mutex\n");
			}
		}
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

void ssd1306_task() {
	uint8_t len = 0;
	for (;;) {
		if (uxQueueMessagesWaiting(w25qxx_queue) != 0) {
			if (xSemaphoreTake(mutex1, portMAX_DELAY) == pdTRUE) {
				response("SSD1306 have mutex\n");
				xQueueReceive(w25qxx_queue, &len, portMAX_DELAY);
				W25Q32_ReadData(w25qxx_output_str, 10 * SECTOR_SIZE, len);
				HAL_UART_Transmit(&huart1, w25qxx_output_str, strlen((char*) w25qxx_output_str), 500);
				xSemaphoreGive(mutex1);
			}
		}
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}
void Handle_Command() {
	if (HAL_GetTick() - uart_last_rcv >= 100 && strlen((char*) uart_buf) > 35) {
		if (strlen((char*) uart_buf) > 40) {
			extract_data(uart_buf, &acc_x, &acc_y, &acc_z, &gyz_x, &gyz_y, &gyz_z, &tem, &gas);
			clear_uart_buf();
			uart_recv = 1;
			response("UART OK\n");
			}
		else {
			response("UART fail\n");
			clear_uart_buf();
		}
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == huart1.Instance) {
		if (uart_buf_cnt < sizeof(uart_buf)) {
			uart_buf[uart_buf_cnt] = uart_chr;
		}
		uart_last_rcv = HAL_GetTick();
		uart_buf_cnt++;
		HAL_UART_Receive_IT(&huart1, &uart_chr, sizeof(uart_chr));
	}
}

void clear_uart_buf() {
	memset(uart_buf, 0, strlen((char*) uart_buf));
	uart_buf_cnt = 0;
}

void extract_data(const uint8_t *input, int *acc_x, int *acc_y, int *acc_z,
					int *gyz_x, int *gyz_y, int *gyz_z, float *tem, float *gas) {
	sscanf((char*) input, "X:%d Y:%d Z:%d X:%d Y:%d Z:%d t:%f g:%f", acc_x, acc_y, acc_z, gyz_x, gyz_y, gyz_z, tem, gas);
}

void compress_data(const int acc_x, const int acc_y, const int acc_z,
		const int gyz_x, const int gyz_y, const int gyz_z, const float tem, const float gas) {
	sprintf((char*) w25qxx_str, "X:%d Y:%d Z:%d X:%d Y:%d Z:%d t:%f g:%f", acc_x, acc_y, acc_z, gyz_x, gyz_y, gyz_z, tem, gas);
}

void response(char *rsp) {
//	sprintf((char*) tx, "acc\nX:%-4d\tY:%-4d\tZ:%-4d\ngyz\nX:%-4d\tY:%-4d\tZ:%-4d\n\ntem:%-3.3f\tgas:-%4.3f\n",
//			acc_x, acc_y, acc_z, gyz_x, gyz_y, gyz_z, tem, gas);
	HAL_UART_Transmit(&huart1, (uint8_t*) rsp, strlen (rsp), 200);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == hspi1.Instance) {
		rx_spi_flg = 1;
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
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
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
	while (1) {
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
