#include "stm32l4xx.h"

void TIM2Config (void);
void delay_us (volatile uint16_t us);
void delay_ms (volatile uint16_t ms);

void TIM2Config (void)
{
	/************** STEPS TO FOLLOW *****************
	1. Enable Timer clock
	2. Set the prescalar and the ARR
	3. Enable the Timer, and wait for the update Flag to set
	************************************************/

	RCC->APB1ENR1 |= (1<<0);
	TIM2->PSC = 4;
	TIM2->CR1 &= ~(1<<7);
	TIM2->ARR = (0xffff);
	TIM2->CR1 |= (1<<0);
	while(!(TIM2->SR & (1<<0)));

}

void delay_us (volatile uint16_t us)
{
	/************** STEPS TO FOLLOW *****************
	1. RESET the Counter
	2. Wait for the Counter to reach the entered value. As each count will take 1 us,
		 the total waiting time will be the required us delay
	************************************************/


	TIM2->CNT &= ~(0xffffffff);
	while(TIM2->CNT < us);
}

void delay_ms (volatile uint16_t ms)
{
	for (volatile uint16_t i = 0;i<ms;i++)
	{
		delay_us(1000); // 1ms delay

	}
}
