#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "stdio.h"

void LED_Init();
void Sen_Init();

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void luxLED(float intensity)
{
    if (intensity < 0.2f)
    {
        GPIOA->ODR |= (1 << 5);
        GPIOA->ODR |= (1 << 6);
        GPIOA->ODR |= (1 << 7);
        GPIOB->ODR |= (1 << 6);

    }
    else if (intensity >= 0.2f && intensity < 0.4f)
    {
        GPIOA->ODR |= (1 << 5);
        GPIOA->ODR |= (1 << 6);
        GPIOA->ODR |= (1 << 7);
        GPIOB->ODR &= ~(1 << 6);
    }
    else if (intensity >= 0.4f && intensity < 0.6f)
    {
        GPIOA->ODR |= (1 << 5);
        GPIOA->ODR |= (1 << 6);
        GPIOA->ODR &= ~(1 << 7);
        GPIOB->ODR &= ~(1 << 6);
    }
    else if (intensity >= 0.6f && intensity < 0.8f)
    {
        GPIOA->ODR |= (1 << 5);
        GPIOA->ODR &= ~(1 << 6);
        GPIOA->ODR &= ~(1 << 7);
        GPIOB->ODR &= ~(1 << 6);
    }
    else if (intensity >= 0.8f)
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
    uint8_t addr = 0x23<<1;

    // Create pointer buff to get bytes of lux
    uint16_t lux = 0;
    uint8_t *buff = (uint8_t *)&lux;

    // Continously H Res
    buff[0] = 0b00010000;

    HAL_I2C_Master_Transmit(&hI2C, addr, buff, 1, HAL_MAX_DELAY);
    HAL_Delay(150);

    while (1) {
      HAL_I2C_Master_Receive(&hI2C, addr, buff, 2, HAL_MAX_DELAY);
      
      // To normalise
      float intensity = lux/ 65535.0f;
      HAL_Delay(150);

      luxLED(intensity);
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
