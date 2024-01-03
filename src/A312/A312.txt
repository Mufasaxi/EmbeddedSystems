#include "stm32f4xx_hal.h"

#define LED_PIN GPIO_PIN_5
#define LED_GPIO_PORT GPIOA
#define LED_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

void LED_Init();

int main(void) {
    HAL_Init();
    LED_Init();

    int isButtonOnFlag = 1;
    while (1) {
        // If B1 (Blue Button) Pin is off (set to 0) adjust the flag accordingly
        if ((GPIOC->IDR & (1 << 13)) == 0){
            // If B1 (Blue Button) Pin is off (set to 0) do nothing, until button pressed
            while((GPIOC->IDR & (1 << 13)) == 0){}
            if (isButtonOnFlag == 1) {
                isButtonOnFlag = 0;
            } else {
                isButtonOnFlag = 1;
            }
        }
        // Turning LED 2 on and off based on button flag
        if (isButtonOnFlag == 1) {
            GPIOA->ODR |= (1 << 5);
            HAL_Delay(100);
            GPIOA->ODR &= ~(1 << 5);
            HAL_Delay(100);
        } else {
            GPIOA->ODR &= ~(1 << 5);
        }
    }
}

void LED_Init() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    // Ensure that LED2 Pin is 0, then set it to 1
    GPIOA->MODER &= ~(3 << (5 * 2));
    GPIOA->MODER |= (1 << (5 * 2));

    // Pull up and Push down set to 0, Output speed register set to 11 (High Speed)
    GPIOA->PUPDR &= ~(3 << (5 * 2));
    GPIOA->OSPEEDR |= (3 << (5 * 2));
}

void SysTick_Handler(void) {
    HAL_IncTick();
}
