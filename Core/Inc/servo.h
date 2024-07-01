#ifndef SERVO_H
#define SERVO_H
#include "stm32f1xx_hal.h"

#define SERVO_PWM_TIM  				TIM2
#define SERVO_PWM_CHANNEL			TIM_CHANNEL_1
#define SERVO_ANGLE_OFFSET		(float)0
	
#define ANGLE_RELEASE			(float)-22
#define ANGLE_PRESS				(float)-8
	
typedef struct
{
	float anlge;
	float angle_offset;
	TIM_TypeDef * pwm_tim;
	uint32_t 	  tim_channel;
}SERVO_STRUCT;

extern SERVO_STRUCT servo;
void servo_init(void);
void servo_rotate(SERVO_STRUCT *servo, float angle);
void servo_pen_release(void);
void servo_pen_press(void);
#endif
