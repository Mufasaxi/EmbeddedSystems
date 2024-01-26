// #include "stm32f4xx_hal.h"

// /* Define shift register pins used for seven-segment display */
// #define LATCH_DIO_PIN GPIO_PIN_5
// #define CLOCK_PIN GPIO_PIN_8
// #define DATA_PIN GPIO_PIN_9

// #define POTENTIOMETER_PIN GPIO_PIN_0

// /* Segment byte maps for numbers 0 to 9 */
// const uint8_t SEGMENT_MAP[] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0X80, 0X90};
// /* Byte maps to select digit 1 to 4 */
// const uint8_t SEGMENT_SELECT[] = {0xF1, 0xF2, 0xF4, 0xF8};

// ADC_HandleTypeDef hadc1;

// void SystemClock_Config(void);
// static void MX_GPIO_Init(void);
// static void MX_ADC1_Init(void);
// void sendToSeg(uint8_t segNo, uint8_t hexVal);
// void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t value);
// void displayHex(uint16_t hexValue);
// void Reg_Init();

// uint8_t patterns[16] = {0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90, 0x88, 0x83, 0xC6, 0xA1, 0x84, 0x8E};

// uint8_t select_seg1 = 0xF1 ;
// uint8_t select_seg2 = 0xF2 ;
// uint8_t select_seg3 = 0xF4 ;
// uint8_t select_seg4 = 0xF8 ;

// int main(void)
// {
//   HAL_Init();
//   Reg_Init();  
//   SystemClock_Config();
//   MX_GPIO_Init();
//   MX_ADC1_Init();

//   uint32_t potValue;
//   displayHex(0x0000);

//   while (1)
//   {
//     // HAL_ADC_Start(&hadc1);
//     // if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
//     // {
//     //   potValue = HAL_ADC_GetValue(&hadc1);
//     //   HAL_ADC_Stop(&hadc1);
//     // }
//     potValue = HAL_ADC_GetValue(&hadc1);
//     HAL_ADC_Stop(&hadc1);

//     /* Update the display with the current potentiometer value */
//     // WriteNumberToSegment(0, potValue / 1000);
//     // WriteNumberToSegment(1, (potValue / 100) % 10);
//     // WriteNumberToSegment(2, (potValue / 10) % 10);
//     // WriteNumberToSegment(3, potValue % 10);
//     // displayHex(potValue);
//     displayHex(1234);

//   }
// }

// void Reg_Init() {
//   __HAL_RCC_GPIOA_CLK_ENABLE();
//   __HAL_RCC_GPIOB_CLK_ENABLE();
//   // Chip Select
//   GPIOB->MODER &= ~(3<<(5*2));
//   GPIOB->MODER |= (1<<(5*2));
//   GPIOB->PUPDR &= ~(3<<(5*2));
//   GPIOB->OSPEEDR |= (3<<(5*2));
//   // Set to 1 initially
//   GPIOB->ODR |= (1<<(5));

//   // Clock
//   GPIOA->MODER &= ~(3<<(8*2));
//   GPIOA->MODER |= (1<<(8*2));
//   GPIOA->PUPDR &= ~(3<<(8*2));
//   GPIOA->OSPEEDR |= (3<<(8*2));
//   // Set to 0 initially
//   GPIOA->ODR &= ~(1<<(8));

//   // SDI / DO
//   GPIOA->MODER &= ~(3<<(9*2));
//   GPIOA->MODER |= (1<<(9*2));
//   GPIOA->PUPDR &= ~(3<<(9*2));
//   GPIOA->OSPEEDR |= (3<<(9*2));

// }

// void sendToSeg(uint8_t segNo, uint8_t hexVal)
// {
//   // Latch down (low)
//   GPIOB->ODR &= ~(1<<5);


//   // Transfer Segmenent data
//   shiftOut(DATA_PIN, CLOCK_PIN, 1, hexVal);
//   // Transfer Segmenent data
//   shiftOut(DATA_PIN, CLOCK_PIN, 1, segNo);

//   // Latch up (high) 
//   GPIOB->ODR |= (1<<5);
// }

// void displayHex(uint16_t hexValue) {
//   // Extract individual digits
//   uint8_t digit4 = (hexValue >> 12) & 0xF;
//   uint8_t digit3 = (hexValue >> 8) & 0xF;
//   uint8_t digit2 = (hexValue >> 4) & 0xF;
//   uint8_t digit1 = hexValue & 0xF;

//   // Display each digit on the 7-segment display
//   sendToSeg(select_seg1, patterns[digit4]);
//   sendToSeg(select_seg2, patterns[digit3]);
//   sendToSeg(select_seg3, patterns[digit2]);
//   sendToSeg(select_seg4, patterns[digit1]);
// }

// void WriteNumberToSegment(uint8_t segment, uint8_t value)
// {
//   HAL_GPIO_WritePin(GPIOB, LATCH_DIO_PIN, GPIO_PIN_RESET);
//   shiftOut(DATA_PIN, CLOCK_PIN, 1, SEGMENT_MAP[value]);
//   shiftOut(DATA_PIN, CLOCK_PIN, 1, SEGMENT_SELECT[segment]);
//   HAL_GPIO_WritePin(GPIOB, LATCH_DIO_PIN, GPIO_PIN_SET);
// }

// /* Initialize ADC peripheral */
// static void MX_ADC1_Init(void)
// {
//   ADC_ChannelConfTypeDef sConfig = {0};

//   __HAL_RCC_ADC1_CLK_ENABLE();

//   /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
//   */
//   hadc1.Instance = ADC1;
//   hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2; // ADC_CLOCK_SYNC_PCLK_DIV4;
//   hadc1.Init.Resolution = ADC_RESOLUTION_12B;
//   hadc1.Init.ScanConvMode = DISABLE;
//   hadc1.Init.ContinuousConvMode = DISABLE;
//   hadc1.Init.DiscontinuousConvMode = DISABLE;
//   hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
//   hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
//   hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
//   hadc1.Init.NbrOfConversion = 1;
//   hadc1.Init.DMAContinuousRequests = DISABLE;
//   hadc1.Init.EOCSelection = DISABLE; // ADC_EOC_SINGLE_CONV;
//   if (HAL_ADC_Init(&hadc1) != HAL_OK)
//   {
//     Error_Handler();
//   }

//   /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//   */
//   sConfig.Channel = ADC_CHANNEL_0;
//   sConfig.Rank = 1;
//   sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES; // ADC_SAMPLETIME_3CYCLES;
//   if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }

// /* System Clock Configuration */
// void SystemClock_Config(void)
// {
//   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//   /** Configure the main internal regulator output voltage
//   */
//   __HAL_RCC_PWR_CLK_ENABLE();
//   __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2); // PWR_REGULATOR_VOLTAGE_SCALE3
  
//   /** Initializes the CPU, AHB and APB busses clocks
//   */
//   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//   RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE; // RCC_PLL_ON;
//   // RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//   // RCC_OscInitStruct.PLL.PLLM = 16;
//   // RCC_OscInitStruct.PLL.PLLN = 336;
//   // RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
//   // RCC_OscInitStruct.PLL.PLLQ = 2;
//   // RCC_OscInitStruct.PLL.PLLR = 2;
//   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /** Initializes the CPU, AHB and APB busses clocks
//   */
//   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
//   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI; // RCC_SYSCLKSOURCE_PLLCLK;
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; // RCC_HCLK_DIV2;
//   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }

// static void MX_GPIO_Init(void)
// {
//   GPIO_InitTypeDef GPIO_InitStruct = {0};

//   __HAL_RCC_GPIOB_CLK_ENABLE();
// 	// __HAL_RCC_GPIOA_CLK_ENABLE();


//   /*Configure GPIO pin Output Level */
//   HAL_GPIO_WritePin(GPIOB, LATCH_DIO_PIN, GPIO_PIN_RESET);
// 	// HAL_GPIO_WritePin(GPIOA, LD2_Pin | CLOCK_PIN_Pin | DATA_PIN_Pin, GPIO_PIN_RESET);


//   /*Configure GPIO pins : LATCH_DIO_PIN CLOCK_PIN DATA_PIN */
//   GPIO_InitStruct.Pin = LATCH_DIO_PIN | CLOCK_PIN | DATA_PIN;
//   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//   /*Configure GPIO pin : POTENTIOMETER_PIN */
//   GPIO_InitStruct.Pin = POTENTIOMETER_PIN;
//   GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
// }

// void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t value) {
//   for (uint8_t i = 0; i < 8; i++) {
//     // MSBFIRST: Send the most significant bit (MSB) first, here as input 1
//     // LSBFIRST: Send the least significant bit (LSB) first, here as input 0
//     uint8_t bit = (bitOrder == 1) ? ((value & 0x80) >> 7) : (value & 0x01);

//     // Set data pin   
//     if (bit == 1) {
//       GPIOA->ODR |= (1 << 9);
//     } else {
//       GPIOA->ODR &= ~(1 << 9);
//     }    

//     // Pulse clock (set and reset)
//     GPIOA->ODR |= (1 << 8);
//     GPIOA->ODR &= ~(1 << 8);

//     // Shift value to the next bit
//     value = (bitOrder == 1) ? (value << 1) : (value >> 1);
//   }
// }

// void Error_Handler(void)
// {
//   // Add your error handling code here
// 	__disable_irq();
//   while (1)
//   {
//   }
// }
