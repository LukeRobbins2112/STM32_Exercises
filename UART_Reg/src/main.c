
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "diag/Trace.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "usart.h"
#include "shell.h"
#include "sys_tick.h"
#include "timer.h"
#include "i2c.h"
#include "rtc.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  SystemClock_Config();

  // Freeze timers on debug
  __HAL_DBGMCU_FREEZE_TIM1();    // Enable Timer1 Freeze on Debug
  __HAL_DBGMCU_FREEZE_TIM2();    // Enable Timer2 Freeze on Debug
  __HAL_DBGMCU_FREEZE_TIM3();    // Enable Timer3 Freeze on Debug
  __HAL_DBGMCU_FREEZE_TIM4();    // Enable Timer4 Freeze on Debug

  // Set up SysTick -- fire every 1/4 second
  init_sys_tick(16000000);

  // Enable clocks
  RCC->APB2ENR |= (0x1 << 2); // GPIOA Enabled
  RCC->APB1ENR |= (0x1 << 17); // USART 2 Enabled
  RCC->APB1ENR |= (0x1 << 0); // TIM 2 Enabled

  TimerInit_t init = {.count = 0xFFFE, .prescaler = 5000, .direction = COUNT_DOWN, .pulse_mode = ONE_SHOT, .timer_action = INTERRUPT};
  init_timer(&init);

  // Set up I2C
  I2C_Init_t i2c_init_master;
  i2c_init_master.role = I2C_MASTER;
  i2c_init_master.clk_stretch = STRETCH;
  i2c_init(&i2c_init_master);

  // Test transaction
  uint8_t hour, min, sec;
  rtc_get_time(&hour, &min, &sec);
  char time_buf[6];
  snprintf(time_buf, 6, "%d:%d", min, sec);


  // Set up / configure GPIOA
  // PA2 = output, alternate function push/pull
  GPIOA->CRL = 0x33334933;
  GPIOA->ODR |= (0x1 << 3); // Pullup Input GPIO for TX

  // Configure USART -- interrupt on RX
  // Do not interrupt on TXE -- only do that when starting transmission, then disable
  USART2->CR1 = 0x202C;

  // Baud Rate -- USART2 uses PCLK1
  USART2->BRR = 0x116; // 17.375 =~ 17.36 --> 115200 baud = (32 Mhz) / (16 * 17.36)

  // Enable USART2 IRQ
  NVIC->ISER[1] = (1 << (USART2_IRQn & 0x1F));

  sendData(PROMPT, 8);
  sendData(time_buf, strlen(time_buf));

  initialize_shell();

  while (1)  {
	  process_next_cmd();
  }


}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
