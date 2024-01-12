#include "stm32f4xx_hal.h"


void LED_Init();

int main(void) {
  HAL_Init();
  LED_Init();
  Zaehler();
  
}

//uint16_t  a=0;
uint16_t  a,i,c,d,e,f;
void LED_Init() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIOB->MODER &= ~(3<<(5*2));
  GPIOB->MODER |= (1<<(5*2));
  GPIOB->PUPDR &= ~(3<<(5*2));
  GPIOB->OSPEEDR |= (3<<(5*2));
  GPIOA->MODER &= ~(3<<(8*2));
  GPIOA->MODER |= (1<<(8*2));
  GPIOA->PUPDR &= ~(3<<(8*2));
  GPIOA->OSPEEDR |= (3<<(8*2));
  GPIOA->MODER &= ~(3<<(9*2));
  GPIOA->MODER |= (1<<(9*2));
  GPIOA->PUPDR &= ~(3<<(9*2));
  GPIOA->OSPEEDR |= (3<<(9*2));

}


void Zaehler()
{

  set_digit(0);
  a=0;
  while(1){
    //HAL_Delay(1);
    set_digit(a);
    if ((GPIOA->IDR & (1<<1))==0){
      while((GPIOA->IDR & (1<<1))==0){}
      a++;
      //set_digit(a);
    }
    if((GPIOA-> IDR & (1<<4))==0){
      while((GPIOA->IDR & (1<<4))==0){}
      if(a==0){
        a=65535;
      }
      else{
        a--;
      }
      //set_digit(a);
    }
    if((GPIOB-> IDR & (1<<0))==0){
      while((GPIOB->IDR & (1<<0))==0){}
      a=0;
      //set_digit(a);
    }
  }
}


void SysTick_Handler(void) {
  HAL_IncTick();
}