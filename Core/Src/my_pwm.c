#include "my_pwm.h"

void PWM_SetFrequency(TIM_TypeDef * tim, int freq)
{
	tim->PSC = SystemCoreClock/1000000 - 1;
	tim->ARR = SystemCoreClock/((tim->PSC+1)*freq) - 1;
}

void PWM_SetDuty(TIM_TypeDef * tim, int channel, uint16_t pusle_width)
{
	if(channel == TIM_CHANNEL_1)
	{
		tim->CCR1 = pusle_width;
	}
	else if(channel == TIM_CHANNEL_2)
	{
		tim->CCR2 = pusle_width;
	}
	else if(channel == TIM_CHANNEL_3)
	{
		tim->CCR3 = pusle_width;
	}
	if(channel == TIM_CHANNEL_4)
	{
		tim->CCR4 = pusle_width;
	}	
}
void PWM_Enable(TIM_TypeDef * tim, int channel)
{
	tim->CR1|= TIM_CR1_CEN;
	tim->BDTR|= TIM_BDTR_MOE;
	if(channel == TIM_CHANNEL_1)
	{
		tim->CCER |= TIM_CCER_CC1E;
	}
	else if(channel == TIM_CHANNEL_2)
	{
		tim->CCER |= TIM_CCER_CC2E;
	}
	else if(channel == TIM_CHANNEL_3)
	{
		tim->CCER |= TIM_CCER_CC3E;
	}
	else if(channel == TIM_CHANNEL_4)
	{
		tim->CCER |= TIM_CCER_CC4E;
	}
	else
	if(channel == TIM_CHANNEL_ALL)
	{
		tim->CCER |= TIM_CCER_CC1E;
		tim->CCER |= TIM_CCER_CC2E;
		tim->CCER |= TIM_CCER_CC3E;
		tim->CCER |= TIM_CCER_CC4E;
	}	
}

void PWM_Disable(TIM_TypeDef * tim, int channel)
{
	if(channel == TIM_CHANNEL_1)
	{
		tim->CCER &= ~TIM_CCER_CC1E;
	}
	else if(channel == TIM_CHANNEL_2)
	{
		tim->CCER &= ~TIM_CCER_CC2E;
	}
	else if(channel == TIM_CHANNEL_3)
	{
		tim->CCER &= ~TIM_CCER_CC3E;
	}
	else if(channel == TIM_CHANNEL_4)
	{
		tim->CCER &= ~TIM_CCER_CC4E;
	}
	else
	if(channel == TIM_CHANNEL_ALL)
	{
		tim->CR1 &= ~TIM_CR1_CEN;
		tim->BDTR &= ~TIM_BDTR_MOE;
		tim->CCER &= ~TIM_CCER_CC1E;
		tim->CCER &= ~TIM_CCER_CC2E;
		tim->CCER &= ~TIM_CCER_CC3E;
		tim->CCER &= ~TIM_CCER_CC4E;
	}		
}
