#ifndef MY_PWM_H
#define MY_PWM_H
#include "stm32f1xx_hal.h"

void PWM_SetFrequency(TIM_TypeDef * tim, int freq);
void PWM_SetDuty(TIM_TypeDef * tim, int channel, uint16_t pusle_width);
void PWM_Enable(TIM_TypeDef * tim, int channel);
void PWM_Disable(TIM_TypeDef * tim, int channel);
#endif
