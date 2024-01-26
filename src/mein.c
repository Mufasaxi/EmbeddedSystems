// #include "stm32f4xx_hal.h"
// #include "stm32f4xx_hal_adc.h"

// #include <stdio.h>
// #include <string.h>

// #define FALSE 0
// #define TRUE 1

// #define LATCH_PIN_Pin GPIO_PIN_5
// #define CLOCK_PIN_Pin GPIO_PIN_8
// #define DATA_PIN_Pin GPIO_PIN_9

// ADC_HandleTypeDef hadc1;
// UART_HandleTypeDef huart2;

// // SysClock
// void SystemClock_Config(void);
// // Inits
// static void MX_GPIO_Init(void);
// static void MX_USART2_UART_Init(void);
// static void MX_ADC1_Init(void);

// // adc
// uint32_t adc_val;
// void ADC_IRQHandler(void) {
// 	/* USER CODE BEGIN ADC_IRQn 0 */
// 	/* USER CODE BEGIN ADC_IRQn 1 */
// 	HAL_ADC_IRQHandler(&hadc1);
// 	adc_val = HAL_ADC_GetValue(&hadc1);
// }


// void seven_seg_show(uint8_t seg_num, uint8_t val) {
// 	const uint8_t SEG_SELECTOR[] = { 0xf1, 0xf2, 0xf4, 0xf8 };

// 	// latch off
// 	HAL_GPIO_WritePin(GPIOB, LATCH_PIN_Pin, GPIO_PIN_RESET);

// 	// send val
// 	for (int i = 7; i >= 0; --i) {
// 		HAL_GPIO_WritePin(GPIOA, DATA_PIN_Pin, (val >> i) & 1);
// 		HAL_GPIO_WritePin(GPIOA, CLOCK_PIN_Pin, GPIO_PIN_RESET);
// 		HAL_GPIO_WritePin(GPIOA, CLOCK_PIN_Pin, GPIO_PIN_SET);
// 	}

// 	// send seg select
// 	for (int i = 7; i >= 0; --i) {
// 		HAL_GPIO_WritePin(GPIOA, DATA_PIN_Pin, (SEG_SELECTOR[seg_num] >> i) & 1);
// 		HAL_GPIO_WritePin(GPIOA, CLOCK_PIN_Pin, GPIO_PIN_RESET);
// 		HAL_GPIO_WritePin(GPIOA, CLOCK_PIN_Pin, GPIO_PIN_SET);
// 	}

// 	// latch on
// 	HAL_GPIO_WritePin(GPIOB, LATCH_PIN_Pin, GPIO_PIN_SET);
// }

// void display_int(uint16_t val) {
// 	uint8_t SEG_MAP[] = { 
//       0xc0, // 0
// 			0xf9, // 1
// 			0xa4, // 2
// 			0xb0, // 3
// 			0x99, // 4
// 			0x92, // 5
// 			0x82, // 6
// 			0xf8, // 7
// 			0x80, // 8
// 			0x90  // 9
// 			};

// 	seven_seg_show(0, SEG_MAP[val / 1000]);
// 	seven_seg_show(1, SEG_MAP[(val / 100) % 10]);
// 	seven_seg_show(2, SEG_MAP[(val / 10) % 10]);
// 	seven_seg_show(3, SEG_MAP[val % 10]);
// }

// void display_int_decipoint(uint16_t val) {
// 	uint8_t SEG_MAP[] = { 0xc0, // 0
// 			0xf9, // 1
// 			0xa4, // 2
// 			0xb0, // 3
// 			0x99, // 4
// 			0x92, // 5
// 			0x82, // 6
// 			0xf8, // 7
// 			0x80, // 8
// 			0x90, // 9
// 			};

// 	seven_seg_show(0, SEG_MAP[val / 1000] & 0b01111111); // with Decimal point
// 	seven_seg_show(1, SEG_MAP[(val / 100) % 10]);
// 	seven_seg_show(2, SEG_MAP[(val / 10) % 10]);
// 	seven_seg_show(3, SEG_MAP[val % 10]);
// }



// int main(void) {
//   HAL_Init();

//   // system clock
//   SystemClock_Config();

//   // Inits
//   MX_GPIO_Init();
//   MX_USART2_UART_Init();
//   MX_ADC1_Init();

// 	char msg[20];
// 	uint32_t tick;
// 	uint8_t in_volt = TRUE; // Eine Boolesche Variable, die angibt, ob der Wert in Volt umgerechnet werden soll.
// 	uint32_t adc_reading;
   
// 	while (1) {
// 		/*
// 		 // Get ADC value
// 		 HAL_ADC_Start(&hadc1);
// 		 if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
// 		 adc_val = HAL_ADC_GetValue(&hadc1);
// 		 }
// 		 */

// 		// Convert to volt (0V -> 3.3V = 0->2^12-1 == 4095)
// 		adc_reading = adc_val;
// 		if (in_volt) {
// 			adc_reading = adc_val * 3300 / 4095; // in Millivolt umgerechnet.
// 		}

// 		// Startet den ADC und aktiviert Interrupts
// 		HAL_ADC_Start_IT(&hadc1);

// 		//  ticker aktualisiert
// 		tick = HAL_GetTick();

// 		// Convert and print over serial port
// 		if (tick % 100 == 0) { // every 100ms
// 			if (in_volt)
// 				sprintf(msg, "%lumV == %lu.%luV\r\n", adc_reading, adc_reading / 1000, adc_reading % 1000);
// 			else
// 				sprintf(msg, "%lu\r\n", adc_reading);

// 			HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 10);

// 			if (status != HAL_OK) {
// 				// Error occurred while transmitting
// 			}
// 		}
// 		// auf einem 7-Segment-Display angezeigt
// 		if (in_volt)
// 			display_int_decipoint(adc_reading);
// 		else
// 			display_int(adc_val);
// 	}
// }

// /****************** cubeMX******************************
//  * 
// */

// /**
//   * @brief System Clock Configuration
//   * @retval None
//   */
// void SystemClock_Config(void)
// {
//   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//   /** Configure the main internal regulator output voltage
//   */
//   __HAL_RCC_PWR_CLK_ENABLE();
//   __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
//   /** Initializes the RCC Oscillators according to the specified parameters
//   * in the RCC_OscInitTypeDef structure.
//   */
//   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//   RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//   RCC_OscInitStruct.PLL.PLLM = 16;
//   RCC_OscInitStruct.PLL.PLLN = 336;
//   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
//   RCC_OscInitStruct.PLL.PLLQ = 2;
//   RCC_OscInitStruct.PLL.PLLR = 2;
//   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /** Initializes the CPU, AHB and APB buses clocks
//   */
//   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }

// static void MX_ADC1_Init(void) {
// 	ADC_ChannelConfTypeDef sConfig = { 0 };

// 	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
// 	 */
// 	hadc1.Instance = ADC1;
// 	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
// 	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
// 	hadc1.Init.ScanConvMode = DISABLE;
// 	hadc1.Init.ContinuousConvMode = DISABLE;
// 	hadc1.Init.DiscontinuousConvMode = DISABLE;
// 	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
// 	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
// 	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
// 	hadc1.Init.NbrOfConversion = 1;
// 	hadc1.Init.DMAContinuousRequests = DISABLE;
// 	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
// 	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
// 		Error_Handler();
// 	}
// 	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
// 	 */
// 	sConfig.Channel = ADC_CHANNEL_0;
// 	sConfig.Rank = 1;
// 	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
// 	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
// 		Error_Handler();
// 	}
// }

// /**
//   * @brief USART2 Initialization Function
//   * @param None
//   * @retval None
//   */
// static void MX_USART2_UART_Init(void)
// {
//   huart2.Instance = USART2;
//   huart2.Init.BaudRate = 115200;
//   huart2.Init.WordLength = UART_WORDLENGTH_8B;
//   huart2.Init.StopBits = UART_STOPBITS_1;
//   huart2.Init.Parity = UART_PARITY_NONE;
//   huart2.Init.Mode = UART_MODE_TX_RX;
//   huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//   huart2.Init.OverSampling = UART_OVERSAMPLING_16;
//   if (HAL_UART_Init(&huart2) != HAL_OK)
//   {
//     Error_Handler();
//   } 
// }

// /**
//   * @brief GPIO Initialization Function
//   * @param None
//   * @retval None
//   */
// static void MX_GPIO_Init(void)
// {
// 	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

// 	/* GPIO Ports Clock Enable */
// 	__HAL_RCC_GPIOC_CLK_ENABLE();
// 	__HAL_RCC_GPIOH_CLK_ENABLE();
// 	__HAL_RCC_GPIOA_CLK_ENABLE();
// 	__HAL_RCC_GPIOB_CLK_ENABLE();

// 	/*Configure GPIO pin Output Level */
// 	HAL_GPIO_WritePin(GPIOA, CLOCK_PIN_Pin | DATA_PIN_Pin, GPIO_PIN_RESET);

// 	/*Configure GPIO pin Output Level */
// 	HAL_GPIO_WritePin(GPIOB, LATCH_PIN_Pin, GPIO_PIN_RESET);

// 	/*Configure GPIO pin : B1_Pin */
// 	GPIO_InitStruct.Pin = GPIO_PIN_1;
// 	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
// 	GPIO_InitStruct.Pull = GPIO_NOPULL;
// 	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

// 	/*Configure GPIO pins : PA0 PA1 */
// 	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
// 	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
// 	GPIO_InitStruct.Pull = GPIO_NOPULL;
// 	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

// 	/*Configure GPIO pins : LD2_Pin CLOCK_PIN_Pin DATA_PIN_Pin */
// 	GPIO_InitStruct.Pin = CLOCK_PIN_Pin | DATA_PIN_Pin;
// 	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
// 	GPIO_InitStruct.Pull = GPIO_NOPULL;
// 	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
// 	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

// 	/*Configure GPIO pin : LATCH_PIN_Pin */
// 	GPIO_InitStruct.Pin = LATCH_PIN_Pin;
// 	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
// 	GPIO_InitStruct.Pull = GPIO_NOPULL;
// 	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
// 	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
// }

// /**
//   * @brief  This function is executed in case of error occurrence.
//   * @retval None
//   */
// void Error_Handler(void)
// {
//   /* USER CODE BEGIN Error_Handler_Debug */
// 	/* User can add his own implementation to report the HAL error return state */
// 	__disable_irq();
// 	while (1) {
// 	}
//   /* USER CODE END Error_Handler_Debug */
// }

// #ifdef  USE_FULL_ASSERT
// /**
//   * @brief  Reports the name of the source file and the source line number
//   *         where the assert_param error has occurred.
//   * @param  file: pointer to the source file name
//   * @param  line: assert_param error line source number
//   * @retval None
//   */
// void assert_failed(uint8_t *file, uint32_t line)
// {
//   /* USER CODE BEGIN 6 */
//   /* User can add his own implementation to report the file name and line number,
//      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//   /* USER CODE END 6 */
// }
// #endif /* USE_FULL_ASSERT */

// // void SysTick_Handler(void) {
// //   HAL_IncTick();
// // }