#include "stm32f4xx_hal.h"

uint32_t currentLED;

void LED_Init();

int main(void) {
  HAL_Init();
  LED_Init();
}

void LED_Init() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  // Provide Timer 2 with Tact signal
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  
  // Prescaler, Max counter value and interrupt
  TIM2->PSC = 1000;//9000;
  TIM2->ARR = 9999;
  TIM2->DIER |= TIM_DIER_UIE;

  // Enable counter
  TIM2->CR1 |= TIM_CR1_CEN;

  // Set interrupt priority and enable interrupt
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  currentLED = 0;
  // Set up D1
  GPIOA->MODER &= ~(3 << (5 * 2));
  GPIOA->MODER |= (1 << (5 * 2));
  GPIOA->PUPDR &= ~(3 << (5 * 2));
  GPIOA->OSPEEDR |= (3 << (5 * 2));
  // Set up D2
  GPIOA->MODER &= ~(3 << (6 * 2));
  GPIOA->MODER |= (1 << (6 * 2));
  GPIOA->PUPDR &= ~(3 << (6 * 2));
  GPIOA->OSPEEDR |= (3 << (6 * 2));
  // Set up D3
  GPIOA->MODER &= ~(3 << (7 * 2));
  GPIOA->MODER |= (1 << (7 * 2));
  GPIOA->PUPDR &= ~(3 << (7 * 2));
  GPIOA->OSPEEDR |= (3 << (7 * 2));
  // Set up D4
  GPIOB->MODER &= ~(3 << (6 * 2));
  GPIOB->MODER |= (1 << (6 * 2));
  GPIOB->PUPDR &= ~(3 << (6 * 2));
  GPIOB->OSPEEDR |= (3 << (6 * 2));
  // Turn on all LEDS  (AT THE START THEY'RE ALL ON EXPECT D4 THEN THEY START TURNING OFF SEQUENTIALLY AND IN THE NEXT CYCLE THEY TURN ON AS EXPECTED)
  GPIOA->ODR &= ~(1 << 5);
  GPIOA->ODR &= ~(1 << 6);
  GPIOA->ODR &= ~(1 << 7);
  GPIOB->ODR &= ~(1 << 6);

  while (1) {
    // Do nothing
  }
}

void TIM2_IRQHandler(void) {
  // Delete interrupt request
  TIM2->SR &= ~TIM_SR_UIF;

  // D1
  if (currentLED == 0) {
    // Switch LED on and off and move on to next LED
    GPIOB->ODR |= (1 << 6);
    GPIOA->ODR &= ~(1 << 5);
    currentLED = 1;
  }
  // D2
  else if (currentLED == 1) {
    // Switch LED on and off and move on to next LED
    GPIOA->ODR |= (1 << 5);
    GPIOA->ODR &= ~(1 << 6);
    currentLED = 2;
  }
  // D3
  else if (currentLED == 2) {
    // Switch LED on and off and move on to next LED
    GPIOA->ODR |= (1 << 6);
    GPIOA->ODR &= ~(1 << 7);
    currentLED = 3;
  }
  // D4
  else if (currentLED == 3) {
    // Switch LED on and off and move on to next LED
    GPIOA->ODR |= (1 << 7);
    GPIOB->ODR &= ~(1 << 6);
    currentLED = 0;
  }
}

void SysTick_Handler(void) {
  HAL_IncTick();
}