#include "stm32f4xx_hal.h"
#define DATA_PIN  GPIO_PIN_9
#define CLOCK_PIN GPIO_PIN_8

void Reg_Init();
void sendToSeg(uint8_t segNo, uint8_t hexVal);
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t value);
void displayHex(uint16_t hexValue);


const uint8_t C_HEX  = 0xC6;
const uint8_t TWO_HEX  = 0xA4;
const uint8_t P_HEX  = 0x8C;
uint8_t patterns[16] = {0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90, 0x88, 0x83, 0xC6, 0xA1, 0x84, 0x8E};

uint8_t select_seg1 = 0xF1 ;
uint8_t select_seg2 = 0xF2 ;
uint8_t select_seg3 = 0xF4 ;
uint8_t select_seg4 = 0xF8 ;
uint16_t counter;

const uint8_t SEGMENT_SELECT[] = {0xF1,0xF2,0xF4,0xF8};

int main(void) {
  HAL_Init();
  Reg_Init();  
  counter = 0;
  displayHex(0);
  while (1) {
    displayHex(counter);

    // Check if A1 pressed and increment the counter
    if ((GPIOA->IDR & (1<<1))==0){
      while((GPIOA->IDR & (1<<1))==0){}
      counter++;
    }
    // Check if A2 pressed and decrement the counter
    if((GPIOA-> IDR & (1<<4))==0){
      while((GPIOA->IDR & (1<<4))==0){}
      if(counter==0){
        counter=65535;
      }
      else{
        counter--;
      }
    }
    // Check if A3 pressed and reset the counter
    if((GPIOB-> IDR & (1<<0))==0){
      while((GPIOB->IDR & (1<<0))==0){}
      counter=0;
    }
  }
}

void Reg_Init() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  // Chip Select
  GPIOB->MODER &= ~(3<<(5*2));
  GPIOB->MODER |= (1<<(5*2));
  GPIOB->PUPDR &= ~(3<<(5*2));
  GPIOB->OSPEEDR |= (3<<(5*2));
  // Set to initially 1
  GPIOB->ODR |= (1<<(5));

  // Clock
  GPIOA->MODER &= ~(3<<(8*2));
  GPIOA->MODER |= (1<<(8*2));
  GPIOA->PUPDR &= ~(3<<(8*2));
  GPIOA->OSPEEDR |= (3<<(8*2));
  // Set to initially 0
  GPIOA->ODR &= ~(1<<(8));
  // SDI / DO
  GPIOA->MODER &= ~(3<<(9*2));
  GPIOA->MODER |= (1<<(9*2));
  GPIOA->PUPDR &= ~(3<<(9*2));
  GPIOA->OSPEEDR |= (3<<(9*2));

}

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t value) {
  for (uint8_t i = 0; i < 8; i++) {
    // MSBFIRST: Send the most significant bit (MSB) first, here as input 1
    // LSBFIRST: Send the least significant bit (LSB) first, here as input 0
    uint8_t bit = (bitOrder == 1) ? ((value & 0x80) >> 7) : (value & 0x01);

    // Set data pin   
    if (bit == 1) {
      GPIOA->ODR |= (1 << 9);
    } else {
      GPIOA->ODR &= ~(1 << 9);
    }
      // // Set and reset
      // GPIOA->ODR |= (1 << 8);
      // GPIOA->ODR &= ~(1 << 8);
    

    // Pulse clock
    // Set and reset
    GPIOA->ODR |= (1 << 8);
    GPIOA->ODR &= ~(1 << 8);

    // Shift value to the next bit
    value = (bitOrder == 1) ? (value << 1) : (value >> 1);
  }
}

void sendToSeg(uint8_t segNo, uint8_t hexVal)
{
  /* Make Latch pin Low */
  GPIOB->ODR &= ~(1<<5);


  /* Transfer Segmenent data */
  shiftOut(DATA_PIN, CLOCK_PIN, 1, hexVal);
    /* Transfer Segmenent Number  */
  shiftOut(DATA_PIN, CLOCK_PIN, 1, segNo);

    /* Make Latch pin High so the data appear on Latch parallel pins */
  GPIOB->ODR |= (1<<5);
}

void displayHex(uint16_t hexValue) {
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
void SysTick_Handler(void) {
  HAL_IncTick();
}