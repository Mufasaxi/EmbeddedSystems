#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "stdio.h"

#define LED_PIN_1 GPIO_PIN_5
#define LED_PIN_2 GPIO_PIN_6
#define LED_PIN_3 GPIO_PIN_7
#define LED_PIN_4 GPIO_PIN_6

void LED_Init();
void Sen_Init();

void SysTick_Handler(void)
{
    HAL_IncTick();
}

const uint8_t BH1750_POWER_ON = 0x01;
const uint8_t BH1750_RESET = 0x07;
const uint8_t BH1750_CONTINUOUS_HIGH_RES_MODE = 0x10;

void displayLux(float percentage)
{
    if (percentage < 0.2f)
    {
        // HAL_GPIO_WritePin(GPIOA, LED_PIN_1, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(GPIOA, LED_PIN_2, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(GPIOA, LED_PIN_3, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(GPIOB, LED_PIN_4, GPIO_PIN_SET);

        GPIOA->ODR |= (1 << 5);
        GPIOA->ODR |= (1 << 6);
        GPIOA->ODR |= (1 << 7);
        GPIOB->ODR |= (1 << 6);

    }
    else if (percentage >= 0.2f && percentage < 0.4f)
    {
        // HAL_GPIO_WritePin(GPIOA, LED_PIN_1, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(GPIOA, LED_PIN_2, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(GPIOA, LED_PIN_3, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(GPIOB, LED_PIN_4, GPIO_PIN_RESET);

        GPIOA->ODR |= (1 << 5);
        GPIOA->ODR |= (1 << 6);
        GPIOA->ODR |= (1 << 7);
        GPIOB->ODR &= ~(1 << 6);
    }
    else if (percentage >= 0.4f && percentage < 0.6f)
    {
        // HAL_GPIO_WritePin(GPIOA, LED_PIN_1, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(GPIOA, LED_PIN_2, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(GPIOA, LED_PIN_3, GPIO_PIN_RESET);
        // HAL_GPIO_WritePin(GPIOB, LED_PIN_4, GPIO_PIN_RESET);

        GPIOA->ODR |= (1 << 5);
        GPIOA->ODR |= (1 << 6);
        GPIOA->ODR &= ~(1 << 7);
        GPIOB->ODR &= ~(1 << 6);
    }
    else if (percentage >= 0.6f && percentage < 0.8f)
    {
        // HAL_GPIO_WritePin(GPIOA, LED_PIN_1, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(GPIOA, LED_PIN_2, GPIO_PIN_RESET);
        // HAL_GPIO_WritePin(GPIOA, LED_PIN_3, GPIO_PIN_RESET);
        // HAL_GPIO_WritePin(GPIOB, LED_PIN_4, GPIO_PIN_RESET);

        GPIOA->ODR |= (1 << 5);
        GPIOA->ODR &= ~(1 << 6);
        GPIOA->ODR &= ~(1 << 7);
        GPIOB->ODR &= ~(1 << 6);
    }
    else if (percentage >= 0.8f)
    {
    //     HAL_GPIO_WritePin(GPIOA, LED_PIN_1, GPIO_PIN_RESET);
    //     HAL_GPIO_WritePin(GPIOA, LED_PIN_2, GPIO_PIN_RESET);
    //     HAL_GPIO_WritePin(GPIOA, LED_PIN_3, GPIO_PIN_RESET);
    //     HAL_GPIO_WritePin(GPIOB, LED_PIN_4, GPIO_PIN_RESET);

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

    uint8_t addr = 0x23<<1;// 0u;

    uint8_t buff[2];

    // for (uint8_t address = 0x03u; address < 0x78u; address++)
    // {
    //     if (HAL_I2C_IsDeviceReady(&hI2C, address << 1, 3, 10) == HAL_OK)
    //     {
    //         addr = address;
    //     }
    // }

    uint16_t nLUX = 0;
    uint8_t *pBuffer = (uint8_t *)&nLUX;

    // Continously H Res
    buff[0] = 0b00010000;
    pBuffer[0] = 0b00010000;

    HAL_I2C_Master_Transmit(&hI2C, addr, pBuffer, 1, HAL_MAX_DELAY);
    HAL_Delay(150);

    while (1) {
      HAL_I2C_Master_Receive(&hI2C, addr, pBuffer, 2, HAL_MAX_DELAY);
      
      float grade = nLUX / 65535.0f;
      HAL_Delay(150);

      displayLux(grade);
    }

    // if (addr != 0)
    // {
    //     pBuffer[0] = BH1750_POWER_ON;
    //     if (HAL_I2C_Master_Transmit(&hI2C, addr << 1, pBuffer, 1, 100) != HAL_OK)
    //     {
    //         return 1;
    //     }
    //     HAL_Delay(5);

    //     pBuffer[0] = BH1750_RESET;
    //     if (HAL_I2C_Master_Transmit(&hI2C, addr << 1, pBuffer, 1, 100) != HAL_OK)
    //     {
    //         return 1;
    //     }

    //     pBuffer[0] = BH1750_CONTINUOUS_HIGH_RES_MODE;
    //     if (HAL_I2C_Master_Transmit(&hI2C, addr << 1, pBuffer, 1, 100) != HAL_OK)
    //     {
    //         return 1;
    //     }
    //     HAL_Delay(180);

    //     while (1)
    //     {
    //         // Read measurement
    //         if (HAL_I2C_Master_Receive(&hI2C, addr << 1, pBuffer, 2, 100) != HAL_OK)
    //             continue;

    //         float grade = nLUX / 65535.0f;
    //         HAL_Delay(120);
    //         displayLux(grade);
    //     }
    // }

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

// void LED_Init_A(int N)
// {
//     GPIOA->MODER &= ~(3 << (N * 2)); // mode[1:0] für pin N zurücksetzen
//     GPIOA->MODER |= (1 << (N * 2));  // mode[1:0] für pin N auf 01 (GP output) setzen

//     GPIOA->OTYPER &= ~(1 << N); // type für pin N zurücksetzen (push-pull)

//     GPIOA->OSPEEDR &= ~(3 << (N * 2)); // speed[1:0] für pin N zurücksetzen (low speed)

//     GPIOA->PUPDR &= ~(3 << (N * 2)); // pupdr[1:0] für pin N zurücksetzen (GP output + push-pull)
// }

// void LED_Init_B(int N)
// {
//     GPIOB->MODER &= ~(3 << (N * 2)); // mode[1:0] für pin N zurücksetzen
//     GPIOB->MODER |= (1 << (N * 2));  // mode[1:0] für pin N auf 01 (GP output) setzen

//     GPIOB->OTYPER &= ~(1 << N); // type für pin N zurücksetzen (push-pull)

//     GPIOB->OSPEEDR &= ~(3 << (N * 2)); // speed[1:0] für pin N zurücksetzen (low speed)

//     GPIOB->PUPDR &= ~(3 << (N * 2)); // pupdr[1:0] für pin N zurücksetzen (GP output + push-pull)
// }

void LED_Init()
{
    // LED_Init_A(5);
    // LED_Init_A(6);
    // LED_Init_A(7);
    // LED_Init_B(6);
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
  // Turn on all LEDS  (AT THE START THEY'RE ALL ON EXPECT D4 THEN THEY START TURNING OFF SEQUENTIALLY AND IN THE NEXT CYCLE THEY TURN ON AS EXPECTED)
  GPIOA->ODR &= ~(1 << 5);
  GPIOA->ODR &= ~(1 << 6);
  GPIOA->ODR &= ~(1 << 7);
  GPIOB->ODR &= ~(1 << 6);
}
