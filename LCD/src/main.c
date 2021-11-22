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
#include "stm32f103xb.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay_ms(uint16_t time) {
	volatile unsigned long l = 0;
	for (uint16_t i = 0; i < time; i++) {
		for (l = 0; l < 6000; l++) {
		}
	}
}

#define CLEAR_ODR_BIT(PORT, PIN) PORT->ODR &= ~(1 << PIN)
#define SET_ODR_BIT(PORT, PIN) PORT->ODR |= (1 << PIN)
#define SET_BRR_BIT(PORT, PIN) PORT->BRR = (1 << PIN)
#define SET_BSRR_BIT(PORT, PIN) PORT->BSRR = (1 << PIN)

// Used for data pin direction
#define OUTPUT 0x3
#define INPUT 0x8

// Manage RW pin and data pin direction
#define READ_MODE 0
#define WRITE_MODE 1
uint8_t current_mode = WRITE_MODE;

void configPin(GPIO_TypeDef *portName, uint8_t pinNum, uint8_t config) {
	if (pinNum < 8) {
		portName->CRL &= ~(0xF << (4 * pinNum)); // Clear setting
		portName->CRL |= (config << (4 * pinNum));
	} else {
		pinNum -= 8;
		portName->CRH &= ~(0xF << (4 * pinNum)); // Clear setting
		portName->CRH |= (config << (4 * pinNum));
	}
}

typedef struct Pin {
	GPIO_TypeDef *portName;
	uint8_t pinNum;
} Pin_t;

Pin_t data_pins[8] = { { GPIOA, 0 }, { GPIOA, 1 }, { GPIOA, 10 },
		{ GPIOA, 8 }, { GPIOA, 4 }, { GPIOA, 5 }, { GPIOA, 6 }, { GPIOA, 7 } };

static Pin_t RS_PIN = {GPIOC, 0};
static Pin_t RW_PIN = {GPIOC, 1};
static Pin_t E_PIN = {GPIOB, 0};

void set_output(uint8_t digit){

	// Clear previous digit
	for (int i = 0; i < 8; i++){
		SET_BRR_BIT(data_pins[i].portName, data_pins[i].pinNum);
	}

	delay_ms(2);

	for (int i = 0; i < 8; i++) {
		if (digit & (1 << i)) {
			SET_BSRR_BIT(data_pins[i].portName, data_pins[i].pinNum);
		}
	}
}


void put_value_4bit(uint8_t val) {
	// Load the data pins

	GPIOA->BRR = 0x0F; // Clear lower
	GPIOA->BSRR = (val >> 4) & 0x0F;

	// Pulse E pin to set
	SET_BSRR_BIT(E_PIN.portName, E_PIN.pinNum);
	delay_ms(2);
	SET_BRR_BIT(E_PIN.portName, E_PIN.pinNum);
	delay_ms(10);

	GPIOA->BRR = 0x0F; // Clear lower
	GPIOA->BSRR = (val & 0x0F);

	// Pulse E pin to set
	SET_BSRR_BIT(E_PIN.portName, E_PIN.pinNum);
	delay_ms(2);
	SET_BRR_BIT(E_PIN.portName, E_PIN.pinNum);
	delay_ms(10);
}

void put_value(uint8_t val) {
	// Load the data pins
	set_output(val);

	// Pulse E pin to set
	SET_BSRR_BIT(E_PIN.portName, E_PIN.pinNum);
	delay_ms(2);
	SET_BRR_BIT(E_PIN.portName, E_PIN.pinNum);
}

void configureReadWrite(uint8_t mode) {
	if (mode > 1) {
		return;
	}

	if (mode == current_mode) {
		// already set
		return;
	}

	if (mode == READ_MODE) {
		// Set RW to 0x1 for read
		SET_BSRR_BIT(RW_PIN.portName, RW_PIN.pinNum);

		// Configure data pins for input
		for (int i = 0; i < 8; i++){
			configPin(data_pins[i].portName, data_pins[i].pinNum, INPUT);
		}
	} else if (mode == WRITE_MODE) {
		// Set RW to 0x0 for write
		SET_BRR_BIT(RW_PIN.portName, RW_PIN.pinNum);

		// Configure data pins for output
		for (int i = 0; i < 8; i++){
			configPin(data_pins[i].portName, data_pins[i].pinNum, OUTPUT);
		}
	}

	delay_ms(1);
	current_mode = mode;
}

uint8_t getStatusReg() {
	uint8_t status = 0;

	// Set RS to 0x0 for CMD
	SET_BRR_BIT(RS_PIN.portName, RS_PIN.pinNum);

	configureReadWrite(READ_MODE);

	// Pulse E pin to set
	SET_BSRR_BIT(E_PIN.portName, E_PIN.pinNum);
	delay_ms(1);

	for (int i = 0; i < 8; i++) {
		uint8_t bitIsSet = data_pins[i].portName->IDR & (1 << data_pins[i].pinNum);
		status |= bitIsSet ? (1 << i) : 0;
	}

	SET_BRR_BIT(E_PIN.portName, E_PIN.pinNum);
	delay_ms(1);

	configureReadWrite(WRITE_MODE);

	return status;
}

uint32_t busyCount = 0;
void waitForReady() {
	while (getStatusReg() & 0x80) {
		// wait
		busyCount++;
	}
	busyCount = 0;
}

void send_cmd(uint8_t cmd){
	waitForReady();

	// Set RS to 0x0 for CMD
	SET_BRR_BIT(RS_PIN.portName, RS_PIN.pinNum);
	put_value(cmd);
}

void send_data(uint8_t data){
	waitForReady();

	// Set RS to 0x1 for data
	SET_BSRR_BIT(RS_PIN.portName, RS_PIN.pinNum);
	put_value(data);
}

void lcd_init(){
	// After power on, wait 15ms
	delay_ms(50);

	// Set RW to 0 for init
	// Reset E and RS Pins
	SET_BRR_BIT(RS_PIN.portName, RS_PIN.pinNum);
	SET_BRR_BIT(E_PIN.portName, E_PIN.pinNum);
	SET_BRR_BIT(RW_PIN.portName, RW_PIN.pinNum);

	// Write init commands

	// Function set
	send_cmd(0x38); // 0011 1000
	//delay_ms(20);

	// Screen Off
	send_cmd(0x08);
	//delay_ms(5);

	// Clear screen
	send_cmd(0x01);
	//delay_ms(5);

	// Input set (increment), shift
	send_cmd(0x06);
	//delay_ms(5);

	// Screen On
	send_cmd(0x0D);
	//delay_ms(5);

	// Cursor Return
	send_cmd(0x2);
	//delay_ms(5);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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

	// Enable peripherals
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Enable GPIOA
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; // Enable GPIOB
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; // Enable GPIOC

	// Data Pins
	configureReadWrite(WRITE_MODE);

	GPIOA->ODR = 0x0;

	// Control Pins
	configPin(RW_PIN.portName, RW_PIN.pinNum, OUTPUT);
	configPin(RS_PIN.portName, RS_PIN.pinNum, OUTPUT);
	configPin(E_PIN.portName, E_PIN.pinNum, OUTPUT);

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	lcd_init();
	send_data('h');
	send_data('e');
	send_data('l');
	send_data('l');
	send_data('o');
	send_data(' ');
	send_data('w');
	send_data('o');
	send_data('r');
	send_data('l');
	send_data('d');

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
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

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */

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

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin PA8 */
	GPIO_InitStruct.Pin = LD2_Pin | GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
