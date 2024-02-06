#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "stdio.h"

void LED_Init();
void Sen_Init();

void Reg_Init();
void sendToSeg(uint8_t segNo, uint8_t hexVal);
void shiftOut(uint8_t bitOrder, uint8_t value);
void displayHex(uint16_t hexValue);

uint8_t patterns[16] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90, 0x88, 0x83, 0xC6, 0xA1, 0x84, 0x8E};

uint8_t select_seg1 = 0xF1;
uint8_t select_seg2 = 0xF2;
uint8_t select_seg3 = 0xF4;
uint8_t select_seg4 = 0xF8;

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void luxLED(float intensity)
{
  // if (intensity < 0.2f)
  // {
  //     GPIOA->ODR |= (1 << 5);
  //     GPIOA->ODR |= (1 << 6);
  //     GPIOA->ODR |= (1 << 7);
  //     GPIOB->ODR |= (1 << 6);

  // }
  // else if (intensity >= 0.2f && intensity < 0.4f)
  // {
  //     GPIOA->ODR |= (1 << 5);
  //     GPIOA->ODR |= (1 << 6);
  //     GPIOA->ODR |= (1 << 7);
  //     GPIOB->ODR &= ~(1 << 6);
  // }
  // else if (intensity >= 0.4f && intensity < 0.6f)
  // {
  //     GPIOA->ODR |= (1 << 5);
  //     GPIOA->ODR |= (1 << 6);
  //     GPIOA->ODR &= ~(1 << 7);
  //     GPIOB->ODR &= ~(1 << 6);
  // }
  // else if (intensity >= 0.6f && intensity < 0.8f)
  // {
  //     GPIOA->ODR |= (1 << 5);
  //     GPIOA->ODR &= ~(1 << 6);
  //     GPIOA->ODR &= ~(1 << 7);
  //     GPIOB->ODR &= ~(1 << 6);
  // }
  // else if (intensity >= 0.8f)
  // {
  //     GPIOA->ODR &= ~(1 << 5);
  //     GPIOA->ODR &= ~(1 << 6);
  //     GPIOA->ODR &= ~(1 << 7);
  //     GPIOB->ODR &= ~(1 << 6);
  // }
  if (intensity < 0.2f)
  {
    GPIOA->ODR |= (1 << 5);
    GPIOA->ODR |= (1 << 6);
    GPIOA->ODR |= (1 << 7);
    GPIOB->ODR |= (1 << 6);
  }
  else if (intensity >= 1.2f && intensity < 1.4f)
  {
    GPIOA->ODR |= (1 << 5);
    GPIOA->ODR |= (1 << 6);
    GPIOA->ODR |= (1 << 7);
    GPIOB->ODR &= ~(1 << 6);
  }
  else if (intensity >= 1.4f && intensity < 1.6f)
  {
    GPIOA->ODR |= (1 << 5);
    GPIOA->ODR |= (1 << 6);
    GPIOA->ODR &= ~(1 << 7);
    GPIOB->ODR &= ~(1 << 6);
  }
  else if (intensity >= 1.6f && intensity < 1.8f)
  {
    GPIOA->ODR |= (1 << 5);
    GPIOA->ODR &= ~(1 << 6);
    GPIOA->ODR &= ~(1 << 7);
    GPIOB->ODR &= ~(1 << 6);
  }
  else if (intensity >= 1.8f)
  {
    GPIOA->ODR &= ~(1 << 5);
    GPIOA->ODR &= ~(1 << 6);
    GPIOA->ODR &= ~(1 << 7);
    GPIOB->ODR &= ~(1 << 6);
  }
}

int main(void)
{
  HAL_Init();

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  LED_Init();
  Sen_Init();
  Reg_Init();

  I2C_HandleTypeDef hI2C;
  hI2C.Instance = I2C1;
  hI2C.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hI2C.Init.OwnAddress1 = 0;
  hI2C.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hI2C.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hI2C.Init.OwnAddress2 = 0;
  hI2C.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hI2C.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hI2C) != HAL_OK)
  {
    return 0;
  }

  // To get a whole Byte
  uint8_t addr = 0x23 << 1;

  // Create pointer buff to get bytes of lux
  uint16_t lux = 0;
  uint8_t *buff = (uint8_t *)&lux;

  // Continously H Res
  buff[0] = 0b00010000;

  HAL_I2C_Master_Transmit(&hI2C, addr, buff, 1, HAL_MAX_DELAY);
  HAL_Delay(150);
  uint16_t intensity2;

  while (1)
  {
    // displayHex(0x145);
    HAL_I2C_Master_Receive(&hI2C, addr, buff, 2, HAL_MAX_DELAY);

    // To normalise
    float intensity = /*lux/ 1.2f;*/ lux / pow(2,16);//65535.0f;
    // intensity /= 100000.0f;
    displayHex(lux);
    luxLED(intensity);
    // HAL_Delay(150);
  }

  while (1)
  {
  }
}

void Sen_Init()
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void LED_Init()
{
  GPIOA->MODER &= ~(3 << (5 * 2));
  GPIOA->MODER |= (1 << (5 * 2));
  GPIOA->PUPDR &= ~(3 << (5 * 2));
  GPIOA->OSPEEDR &= ~(3 << (5 * 2));
  // Set up D2
  GPIOA->MODER &= ~(3 << (6 * 2));
  GPIOA->MODER |= (1 << (6 * 2));
  GPIOA->PUPDR &= ~(3 << (6 * 2));
  GPIOA->OSPEEDR &= ~(3 << (6 * 2));
  // Set up D3
  GPIOA->MODER &= ~(3 << (7 * 2));
  GPIOA->MODER |= (1 << (7 * 2));
  GPIOA->PUPDR &= ~(3 << (7 * 2));
  GPIOA->OSPEEDR &= ~(3 << (7 * 2));
  // Set up D4
  GPIOB->MODER &= ~(3 << (6 * 2));
  GPIOB->MODER |= (1 << (6 * 2));
  GPIOB->PUPDR &= ~(3 << (6 * 2));
  GPIOB->OSPEEDR &= ~(3 << (6 * 2));
  // Turn off all LEDS
  GPIOA->ODR &= ~(1 << 5);
  GPIOA->ODR &= ~(1 << 6);
  GPIOA->ODR &= ~(1 << 7);
  GPIOB->ODR &= ~(1 << 6);
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

  // Additional ADC GPIO configuration (A0)
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void shiftOut(uint8_t bitOrder, uint8_t value)
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
  shiftOut(1, hexVal);
  // Transfer Segmenent data
  shiftOut(1, segNo);

  // Latch up (high)
  GPIOB->ODR |= (1 << 5);
}

void displayHex(uint16_t hexValue)
{
  // Extract individual digits
  uint8_t digit4 = (hexValue >> 12) & 0xF;
  uint8_t digit3 = (hexValue >> 8) & 0xF;
  uint8_t digit2 = (hexValue >> 4) & 0xF;
  uint8_t digit1 = hexValue & 0xF;

  // Display each digit on the 7-segment display
  sendToSeg(select_seg1, patterns[digit4]);
  sendToSeg(select_seg2, patterns[digit3]);
  sendToSeg(select_seg3, patterns[digit2]);
  sendToSeg(select_seg4, patterns[digit1]);
}