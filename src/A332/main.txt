#include "stm32f4xx_hal.h"
#include <math.h>
#define DATA_PIN GPIO_PIN_9
#define CLOCK_PIN GPIO_PIN_8

ADC_HandleTypeDef hadc1;

void Reg_Init();
void sendToSeg(uint8_t segNo, uint8_t hexVal);
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t value);
void displayHex(uint16_t hexValue);
static void SystemClock_Config(void);
void Error_Handler(void);

const uint8_t C_HEX = 0xC6;
const uint8_t TWO_HEX = 0xA4;
const uint8_t P_HEX = 0x8C;
uint8_t patterns[16] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90, 0x88, 0x83, 0xC6, 0xA1, 0x84, 0x8E};

uint8_t select_seg1 = 0xF1;
uint8_t select_seg2 = 0xF2;
uint8_t select_seg3 = 0xF4;
uint8_t select_seg4 = 0xF8;
// uint16_t counter;

const uint8_t SEGMENT_SELECT[] = {0xF1, 0xF2, 0xF4, 0xF8};


volatile uint16_t adcValue;

// ISR for ADC conversion complete
// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
// {
//   // Read ADC value
//   adcValue = HAL_ADC_GetValue(hadc);

//   // Display ADC value on 7-segment display
//   displayHex(adcValue);
// }

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  Reg_Init();
  __HAL_RCC_ADC1_CLK_ENABLE();

  // Initialize ADC
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    // Initialization Error
    Error_Handler();
  }

  // Configure ADC channel
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    // Configuration Error
    Error_Handler();
  }

  // Start ADC conversion
  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    // ADC Start Error
    Error_Handler();
  }
  // uint16_t adcValue;
  while (1)
  {
    // displayHex(adcValue);
    // Wait for ADC conversion to complete
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
    {
      // Read ADC value
      adcValue = HAL_ADC_GetValue(&hadc1);
      displayHex(adcValue);
      // Display ADC value on 7-segment display
    }
    // displayHex(adcValue);

    // adcValue = HAL_ADC_GetValue(&hadc1);

    // displayHex(adcValue);

  }
}

void Reg_Init()
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  // Chip Select
  GPIOB->MODER &= ~(3 << (5 * 2));
  GPIOB->MODER |= (1 << (5 * 2));
  GPIOB->PUPDR &= ~(3 << (5 * 2));
  GPIOB->OSPEEDR |= (3 << (5 * 2));
  // Set to 1 initially
  GPIOB->ODR |= (1 << (5));

  // Clock
  GPIOA->MODER &= ~(3 << (8 * 2));
  GPIOA->MODER |= (1 << (8 * 2));
  GPIOA->PUPDR &= ~(3 << (8 * 2));
  GPIOA->OSPEEDR |= (3 << (8 * 2));
  // Set to 0 initially
  GPIOA->ODR &= ~(1 << (8));

  // SDI / DO
  GPIOA->MODER &= ~(3 << (9 * 2));
  GPIOA->MODER |= (1 << (9 * 2));
  GPIOA->PUPDR &= ~(3 << (9 * 2));
  GPIOA->OSPEEDR |= (3 << (9 * 2));

  // Additional ADC GPIO configuration (Potentiometer at A0)
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t value)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // MSBFIRST: Send the most significant bit (MSB) first, here as input 1
    // LSBFIRST: Send the least significant bit (LSB) first, here as input 0
    uint8_t bit = (bitOrder == 1) ? ((value & 0x80) >> 7) : (value & 0x01);

    // Set data pin
    if (bit == 1)
    {
      GPIOA->ODR |= (1 << 9);
    }
    else
    {
      GPIOA->ODR &= ~(1 << 9);
    }

    // Pulse clock (set and reset)
    GPIOA->ODR |= (1 << 8);
    GPIOA->ODR &= ~(1 << 8);

    // Shift value to the next bit
    value = (bitOrder == 1) ? (value << 1) : (value >> 1);
  }
}

void sendToSeg(uint8_t segNo, uint8_t hexVal)
{
  // Latch down (low)
  GPIOB->ODR &= ~(1 << 5);

  // Transfer Segmenent data
  shiftOut(DATA_PIN, CLOCK_PIN, 1, hexVal);
  // Transfer Segmenent data
  shiftOut(DATA_PIN, CLOCK_PIN, 1, segNo);

  // Latch up (high)
  GPIOB->ODR |= (1 << 5);
}

void displayHex(uint16_t hexValue)
{
  // Scale ADC value to voltage (assuming a 3.3V reference)
  float voltage = (hexValue / 4095.0) * 3300.0;

  // Extract individual digits
  uint8_t digit1 = voltage / 1000;
  uint8_t digit2 = ((int)voltage / 100) % 10;
  uint8_t digit3 = ((int)voltage / 10) % 10;
  uint8_t digit4 = (int)voltage % 10;

  // Display each digit on the 7-segment display
  sendToSeg(select_seg1, patterns[digit1] & 0b01111111);
  sendToSeg(select_seg2, patterns[digit2]);
  sendToSeg(select_seg3, patterns[digit3]);
  sendToSeg(select_seg4, patterns[digit4]);

  // // Extract individual digits
  // uint8_t digit4 = (hexValue >> 12) & 0xF;
  // uint8_t digit3 = (hexValue >> 8) & 0xF;
  // uint8_t digit2 = (hexValue >> 4) & 0xF;
  // uint8_t digit1 = hexValue & 0xF;

  // // Display each digit on the 7-segment display
  // sendToSeg(select_seg1, patterns[digit4]);
  // sendToSeg(select_seg2, patterns[digit3]);
  // sendToSeg(select_seg3, patterns[digit2]);
  // sendToSeg(select_seg4, patterns[digit1]);
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* Activate the OverDrive to reach the 180 MHz Frequency */  
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}

void Error_Handler(void)
{
  // User-specific error handling code
  while (1)
  {
    // Toggle the LED on and off to indicate an error
    HAL_GPIO_TogglePin(GPIOA, 5);

    // Insert a delay to make the blinking visible
    HAL_Delay(500);
  }
}
