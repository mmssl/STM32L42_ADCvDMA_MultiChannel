#define STM32L452xx 1
#include "stm32l4xx.h"
#include "Delay_ms.h"

volatile float v_lvl[2];
volatile uint16_t adc_val[2];


void ADC_Init(void);
void ADC_enable (void);
void DMA_Init (void);
void DMA_Config (volatile uint32_t srcAdd, volatile uint32_t destAdd, volatile uint8_t size);

int main (void)
{
  SystemInit();
  GPIOB->MODER &= ~(0xffffffff);
  GPIOB->MODER |=  (1<<26);
  TIM2Config();
  ADC_Init();
  ADC_enable();
  DMA_Init();

  while (1)
  {
     ADC1->CR |= (1<<2); // start the ADC
     DMA_Config(( uint32_t) &ADC1->DR, ( uint32_t) adc_val, 2);

     // Converting ADC values to V levels.
     v_lvl[0] = (3.3*adc_val[0])/4095;
     v_lvl[1] = (3.3*adc_val[1])/4095;

  }
return 0;
}

void ADC_Init (void)
{

  RCC->AHB2ENR |=  (1<<2) | (1<<13) | (1<<1); // enable GPIOC clock and ADC clock
  RCC->CCIPR   &= ~(3<<28);
  RCC->CCIPR   |=  (3<<28);

  ADC1->CR &= ~(1<<29); // disable the deep-power-down by Setting DEEPWD to 0
  ADC1->CR |=  (1<<0); // disable ADC
  ADC1->CR |=  (1<<28); // enable the voltage regulator
  delay_ms(10);
  //ADC1->CR |= (2<<18); // divided by 4
  ADC1->CFGR  &= ~(3<<3); // set resolution to 12-bit
  ADC1->CFGR  |=  (1<<13); // continuous conversion mode
  ADC1->CFGR  &= ~(1<<5); // right Alignment
  ADC1->SMPR1 &= ~(7<<3) & ~(7<<12); // reset Sampling rate for P 0 & 3
  ADC1->SMPR1 |=  (7<<3) | (7<<12);  //  640.5 Sampling rate for P 0 & 3
  ADC1->SQR1  &= ~(0xf);
  ADC1->SQR1  |=  (1<<0); // for 2 channel

  GPIOC->MODER &= ~(0xffffffff); // reset the whole register
  GPIOC->MODER |=  (3<<0) | (3<<6); // set the analog mode to pin PC0 and PC3

  ADC1->CFGR |=  (1<<0); // Enable DMA
  ADC1->SQR1 &= ~(0xf<<6) & ~(0xf<<12);
  ADC1->SQR1 |=  (1<<6) | (4<<12);
}

void DMA_Init (void)
{
  RCC->AHB1ENR |= (1<<0); // Enable DMA1 clock

  DMA1_Channel1->CCR &= ~(1<<4); // Set the data direction (P-M)
  DMA1_Channel1->CCR |=  (1<<5); // Circullium mode enable
  DMA1_Channel1->CCR |=  (1<<7); // Enable memory increment Mode
  DMA1_Channel1->CCR |=  (1<<8) | (1<<10); // peripheral and memory size setting
}

void  DMA_Config (volatile uint32_t srcAdd, volatile uint32_t destAdd, volatile uint8_t size)
{

  DMA1_Channel1->CNDTR = size; // size of the transfer
  DMA1_Channel1->CPAR  = srcAdd; // adress of the peripheral
  DMA1_Channel1->CMAR  = destAdd; // adress of the source
  DMA1_Channel1->CCR  |= (1<<0); // enable DMA1

}

void ADC_enable (void)
{
  ADC1->ISR |= (1<<0);
  ADC1->CR  |= (1<<0);
  delay_ms(10);
}
