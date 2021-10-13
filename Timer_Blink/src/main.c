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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "diag/Trace.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WELCOME_MSG "Welcome to the Nucleo management console\r\n"
#define MAIN_MENU   "Select the option you are interested in:\r\n\t1. Toggle LD2 LED\r\n\t2. Read USER BUTTON status\r\n\t3. Clear screen and print this message "
#define PROMPT "\r\n> "
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_timer;
UART_HandleTypeDef huart2;
char readBuf[1];
char recv[5];
uint8_t data[] = { 0xFF, 0x0 };
__IO ITStatus UartReady = SET;

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
void printWelcomeMessage(void);
uint8_t processUserInput(uint8_t opt);
int8_t readUserInput(void);
void performCriticalTasks(void);

void DMA_Complete(DMA_HandleTypeDef *handle);
void DMA_Complete_RX(DMA_HandleTypeDef *handle);

extern void initialise_monitor_handles();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_0) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
	} else if (GPIO_Pin == GPIO_PIN_12) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	initialise_monitor_handles();
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	// Nucleo_BSP_Init();

	/* USER CODE BEGIN Init */

	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	HAL_NVIC_SetPriority(USART2_IRQn, 0x0, 0x0);

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_DMA_Init();
	/* USER CODE BEGIN 2 */

	// Enable timer
	__HAL_RCC_TIM2_CLK_ENABLE();

	HAL_NVIC_SetPriority(TIM2_IRQn, 0x0, 0x0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 63999;
	htim2.Init.Period = 499;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_Base_Init(&htim2);
	HAL_TIM_Base_Start(&htim2);

	// Configure DMA handles
	hdma_usart2_rx.Instance = DMA1_Channel6;
	hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart2_rx.Init.Mode = DMA_NORMAL;
	hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart2_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
	hdma_usart2_rx.XferCpltCallback = &DMA_Complete_RX;
	hdma_usart2_rx.XferAbortCallback = NULL;
	hdma_usart2_rx.XferErrorCallback = NULL;
	hdma_usart2_rx.XferHalfCpltCallback = NULL;
	HAL_DMA_Init(&hdma_usart2_rx);

	// HAL_DMA_Start_IT(&hdma_usart2_rx, (uint32_t)(&huart2.Instance->DR), (uint32_t)readBuf, 1);

	hdma_usart2_tx.Instance = DMA1_Channel7;
	hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart2_tx.Init.Mode = DMA_NORMAL;
	hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart2_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
	hdma_usart2_tx.XferCpltCallback = &DMA_Complete;
	hdma_usart2_tx.XferAbortCallback = NULL;
	hdma_usart2_tx.XferErrorCallback = NULL;
	hdma_usart2_tx.XferHalfCpltCallback = NULL;
	HAL_DMA_Init(&hdma_usart2_tx);

	hdma_timer.Instance = DMA1_Channel2;
	hdma_timer.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_timer.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_timer.Init.MemInc = DMA_MINC_ENABLE;
	hdma_timer.Init.Mode = DMA_CIRCULAR;
	hdma_timer.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_timer.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_timer.Init.Priority = DMA_PRIORITY_MEDIUM;
//	hdma_timer.XferCpltCallback = &DMA_Complete;
//	hdma_timer.XferAbortCallback = NULL;
//	hdma_timer.XferErrorCallback = NULL;
//	hdma_timer.XferHalfCpltCallback = NULL;
	HAL_DMA_Init(&hdma_timer);

	HAL_DMA_Start(&hdma_timer, (uint32_t) data, (uint32_t) &GPIOA->ODR, 2);
	__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_UPDATE);
//
//  const char* msg = "Testing123";
//
//  HAL_DMA_Start_IT(&hdma_usart2_tx, (uint32_t)msg, (uint32_t)(&huart2.Instance->DR), strlen(msg));

//Enable UART in DMA mode
	huart2.Instance->CR3 |= USART_CR3_DMAT;
	huart2.Instance->CR3 |= USART_CR3_DMAR;

	/* USER CODE END 2 */

	while (1) {
	}

	/* USER CODE END 3 */
}

void printWelcomeMessage(void) {
	char *strings[] = { "\033[0;0H", "\033[2J", WELCOME_MSG, MAIN_MENU, PROMPT };

	for (uint8_t i = 0; i < 5; i++) {
		HAL_UART_Transmit_IT(&huart2, (uint8_t*) strings[i],
				strlen(strings[i]));
		// Wait for previous transmit call to finish
		while (HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX
				|| HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX_RX)
			;
	}
}

int8_t readUserInput(void) {
	int8_t retVal = -1;

	if (UartReady == SET) {
		UartReady = RESET;
		HAL_StatusTypeDef res = HAL_UART_Receive_IT(&huart2, (uint8_t*) readBuf,
				1);
		if (res == HAL_OK) {
			retVal = atoi(readBuf);
		}
	}
	return retVal;
}

uint8_t processUserInput(uint8_t opt) {
	char msg[30];

	if (opt < 1 || opt > 3) {
		sprintf(msg, "%s", "invalid");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		return 0;
	}

	sprintf(msg, "%d", opt);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	switch (opt) {
	case 1:
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		break;
	case 2:
		sprintf(msg, "\r\nUSER BUTTON status: %s",
				HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET ?
						"PRESSED" : "RELEASED");
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		break;
	case 3:
		return 2;
	};

	HAL_UART_Transmit(&huart2, (uint8_t*) PROMPT, strlen(PROMPT),
	HAL_MAX_DELAY);
	return 1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
}

void DMA_Complete(DMA_HandleTypeDef *handle) {
	// Undo DMA mode
	//huart2.Instance->CR3 &= ~USART_CR3_DMAT;
	HAL_DMA_Start_IT(&hdma_usart2_rx, (uint32_t) (&huart2.Instance->DR),
			(uint32_t) readBuf, 1);

	// Turn on LED, just to signal we're done
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}

void DMA_Complete_RX(DMA_HandleTypeDef *handle) {
	// Undo DMA mode
	//huart2.Instance->CR3 &= ~USART_CR3_DMAT;
	HAL_DMA_Start_IT(&hdma_usart2_tx, (uint32_t) readBuf,
			(uint32_t) (&huart2.Instance->DR), 1);

	// Turn on LED, just to signal we're done
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
	/* Set transmission flag: transfer complete*/
	UartReady = SET;
}

void performCriticalTasks(void) {
	HAL_Delay(100);
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_DMA_Init(void) {
	// Enable peripheral clock
	__HAL_RCC_DMA1_CLK_ENABLE();

	// Initialize interrupt
	// RX --> channel 6, TX --> channel 7
	HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0x0, 0x0);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0x0, 0x0);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
