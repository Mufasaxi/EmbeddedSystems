#include "stm32f4xx_hal.h"

uint16_t  a,i,c,d,e,f;

uint16_t  b[16][8]={
    {1,1,0,0,0,0,0,0},//0x00
    {1,1,1,1,1,0,0,1},//0x01
    {1,0,1,0,0,1,0,0},//0x02
    {1,0,1,1,0,0,0,0},//0x03
    {1,0,0,1,1,0,0,1},//0x04
    {1,0,0,1,0,0,1,0},//0x05
    {1,0,0,0,0,0,1,0},//0x06
    {1,1,1,1,1,0,0,0},//0x07
    {1,0,0,0,0,0,0,0},//0x08
    {1,0,0,1,0,0,0,0},//0x09
    {1,0,0,0,1,0,0,0},//0x0A
    {1,0,0,0,0,0,1,1},//0x0b
    {1,1,0,0,0,1,1,0},//0x0c
    {1,0,1,0,0,0,0,1},//0x0d
    {1,0,0,0,0,1,1,0},//0x0E
    {1,0,0,0,1,1,1,0},//0x0F
    };

void set_digit(uint16_t a){
  f=a/4096;// vierte Stelle
  e=(a-f*4096)/256;//dritte Stelle
  d=(a-f*4096-e*256)/16; // zweite Stelle
  c=(a-f*4096-e*256-d*16); // erste Stelle

 //while(1){

  set_digit_c(c);
  HAL_Delay(1);
  set_digit_d(d);
  HAL_Delay(1);
  set_digit_e(e);
  HAL_Delay(1);
  set_digit_f(f);
  HAL_Delay(1);
  //}
}
void set_digit_c(uint16_t c){


  for(i=0;i<16;i++){
    if (i<8){
      if(b[c][i]==0){
        GPIOA->ODR &= ~(1<<9);
        GPIOA->ODR |= (1<<8);
        GPIOA->ODR &= ~(1<<8);
       
      }
      else if(b[c][i]==1){
        GPIOA->ODR |= (1<<9);
        GPIOA->ODR |= (1<<8);
        GPIOA->ODR &= ~(1<<8);
        
      }

    }
    if(i==12){
      GPIOA->ODR |= (1<<9);
      GPIOA->ODR |= (1<<8);
      GPIOA->ODR &= ~(1<<8);}
    else if (i!=12 && i>7){
      GPIOA->ODR &= ~(1<<9);
      //GPIOA->ODR |= (1<<9);
      GPIOA->ODR |= (1<<8);
      GPIOA->ODR &= ~(1<<8);
      }
        
  }
  GPIOB->ODR |= (1<<5);
  GPIOB->ODR &= ~(1<<5);
}

void set_digit_d(uint16_t d){


  for(i=0;i<16;i++){
    if (i<8){
      if(b[d][i]==0){
        GPIOA->ODR &= ~(1<<9);
        GPIOA->ODR |= (1<<8);
        GPIOA->ODR &= ~(1<<8);
       
      }
      else if(b[d][i]==1){
        GPIOA->ODR |= (1<<9);
        GPIOA->ODR |= (1<<8);
        GPIOA->ODR &= ~(1<<8);
        
      }

    }
    if(i==13){
      GPIOA->ODR |= (1<<9);
      GPIOA->ODR |= (1<<8);
      GPIOA->ODR &= ~(1<<8);}
    else if (i!=13 && i>7){
      GPIOA->ODR &= ~(1<<9);
      //GPIOA->ODR |= (1<<9);
      GPIOA->ODR |= (1<<8);
      GPIOA->ODR &= ~(1<<8);
      }
        
  }
  GPIOB->ODR |= (1<<5);
  GPIOB->ODR &= ~(1<<5);
}

void set_digit_e(uint16_t e){


  for(i=0;i<16;i++){
    if (i<8){
      if(b[e][i]==0){
        GPIOA->ODR &= ~(1<<9);
        GPIOA->ODR |= (1<<8);
        GPIOA->ODR &= ~(1<<8);
       
      }
      else if(b[e][i]==1){
        GPIOA->ODR |= (1<<9);
        GPIOA->ODR |= (1<<8);
        GPIOA->ODR &= ~(1<<8);
        
      }

    }
    if(i==14){
      GPIOA->ODR |= (1<<9);
      GPIOA->ODR |= (1<<8);
      GPIOA->ODR &= ~(1<<8);}
    else if (i!=14 && i>7){
      GPIOA->ODR &= ~(1<<9);
      //GPIOA->ODR |= (1<<9);
      GPIOA->ODR |= (1<<8);
      GPIOA->ODR &= ~(1<<8);
      }
        
  }
  GPIOB->ODR |= (1<<5);
  GPIOB->ODR &= ~(1<<5);
}

void set_digit_f(uint16_t f){


  for(i=0;i<16;i++){
    if (i<8){
      if(b[f][i]==0){
        GPIOA->ODR &= ~(1<<9);
        GPIOA->ODR |= (1<<8);
        GPIOA->ODR &= ~(1<<8);
       
      }
      else if(b[f][i]==1){
        GPIOA->ODR |= (1<<9);
        GPIOA->ODR |= (1<<8);
        GPIOA->ODR &= ~(1<<8);
        
      }

    }
    if(i==15){
      GPIOA->ODR |= (1<<9);
      GPIOA->ODR |= (1<<8);
      GPIOA->ODR &= ~(1<<8);}
    else if (i!=15 && i>7){
      GPIOA->ODR &= ~(1<<9);
      //GPIOA->ODR |= (1<<9);
      GPIOA->ODR |= (1<<8);
      GPIOA->ODR &= ~(1<<8);
      }
        
  }
  GPIOB->ODR |= (1<<5);
  GPIOB->ODR &= ~(1<<5);
}


