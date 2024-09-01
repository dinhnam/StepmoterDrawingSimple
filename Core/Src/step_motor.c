#include "step_motor.h"
#include <stdio.h>
#include <math.h>
#include "my_pwm.h"

STEPMOTOR_STRUCT stepmotor1, stepmotor2;
int sinValue[STEP_NUMBER];
int cosValue[STEP_NUMBER];
float step_interval = STEP_TIME_MIN;

void AHL(STEPMOTOR_STRUCT *stepmotor, uint16_t pwm_duty)
{
	PWM_SetDuty(TIM1, stepmotor->pwm_channel[0], pwm_duty);
	PWM_Enable(TIM1, stepmotor->pwm_channel[0]);
	HAL_GPIO_WritePin(stepmotor->port, stepmotor->pin[PIN_A2], GPIO_PIN_RESET);
}

void ALH(STEPMOTOR_STRUCT *stepmotor, uint16_t pwm_duty)
{
	PWM_SetDuty(TIM1, stepmotor->pwm_channel[0], TIM1->ARR - pwm_duty);
	PWM_Enable(TIM1, stepmotor->pwm_channel[0]);
	HAL_GPIO_WritePin(stepmotor->port, stepmotor->pin[PIN_A2], GPIO_PIN_SET);
}

void AFL(STEPMOTOR_STRUCT *stepmotor)
{
	PWM_Disable(TIM1, stepmotor->pwm_channel[0]);
	HAL_GPIO_WritePin(stepmotor->port, stepmotor->pin[PIN_A2], GPIO_PIN_RESET);
}

void BHL(STEPMOTOR_STRUCT *stepmotor, uint16_t pwm_duty)
{
	PWM_SetDuty(TIM1, stepmotor->pwm_channel[1], pwm_duty);
	PWM_Enable(TIM1, stepmotor->pwm_channel[1]);
	HAL_GPIO_WritePin(stepmotor->port, stepmotor->pin[PIN_B2], GPIO_PIN_RESET);
}

void BLH(STEPMOTOR_STRUCT *stepmotor, uint16_t pwm_duty)
{
	PWM_SetDuty(TIM1, stepmotor->pwm_channel[1], TIM1->ARR - pwm_duty);
	PWM_Enable(TIM1, stepmotor->pwm_channel[1]);
	HAL_GPIO_WritePin(stepmotor->port, stepmotor->pin[PIN_B2], GPIO_PIN_SET);
}

void BFL(STEPMOTOR_STRUCT *stepmotor)
{
	PWM_Disable(TIM1, stepmotor->pwm_channel[1]);
	HAL_GPIO_WritePin(stepmotor->port, stepmotor->pin[PIN_B2], GPIO_PIN_RESET);
}

void stepmotor_stop(STEPMOTOR_STRUCT *stepmotor)
{
	AFL(stepmotor);
	BFL(stepmotor);
}

void stepmotor_set_step(STEPMOTOR_STRUCT *stepmotor, int step_value)
{
	if(step_value < STEP_NUMBER/4)
	{
		AHL(stepmotor, cosValue[step_value]);
		BHL(stepmotor, sinValue[step_value]);
	}
	else if(step_value < STEP_NUMBER/2)
	{
		ALH(stepmotor, cosValue[step_value]);
		BHL(stepmotor, sinValue[step_value]);
	}
	else if(step_value < 3*STEP_NUMBER/4)
	{
		ALH(stepmotor, cosValue[step_value]);
		BLH(stepmotor, sinValue[step_value]);
	}
	else if(step_value < STEP_NUMBER)
	{
		AHL(stepmotor, cosValue[step_value]);
		BLH(stepmotor, sinValue[step_value]);
	}
}

void stepmotor_increase_step(STEPMOTOR_STRUCT *stepmotor)
{
	stepmotor->step++;
	if(stepmotor->step >= STEP_NUMBER) stepmotor->step = 0;
	stepmotor->step_count++;
	stepmotor_set_step(stepmotor, stepmotor->step);
}
void stepmotor_decrease_step(STEPMOTOR_STRUCT *stepmotor)
{
	stepmotor->step--;
	if(stepmotor->step < 0) stepmotor->step = STEP_NUMBER-1;
	stepmotor->step_count--;
	stepmotor_set_step(stepmotor, stepmotor->step);
}

void stepmotor_set_step_time(STEPMOTOR_STRUCT *stepmotor, float step_time_us)
{ 
	if( step_time_us < STEP_TIME_MIN)  step_time_us = STEP_TIME_MIN;
	stepmotor->timer->ARR = step_time_us/TIM_COUNTER_TIME;
}

void stepmotor_step_start_it(STEPMOTOR_STRUCT *stepmotor)
{
	stepmotor->timer->CNT = stepmotor->timer->ARR;
	stepmotor->timer->DIER |= TIM_IT_UPDATE;
	stepmotor->timer->CR1 |= TIM_CR1_CEN;
	stepmotor->status = MOTOR_RUN;
}

void stepmotor_step_stop_it(STEPMOTOR_STRUCT *stepmotor)
{
	stepmotor->timer->DIER &= ~TIM_IT_UPDATE;
	stepmotor->timer->CR1 &= ~TIM_CR1_CEN;
	stepmotor->status = MOTOR_STOP;
}

void stepmotor_set_step_num(STEPMOTOR_STRUCT *stepmotor, int step_count_set, float step_time)
{
	if( step_count_set != stepmotor->step_count_set)
	{
		stepmotor->step_count_set = step_count_set;
		stepmotor_set_step_time(stepmotor, step_time);
		if(stepmotor->step_count_set != stepmotor->step_count)
		{
			stepmotor_step_start_it(stepmotor);
		}
	}
}

void stepmotor_set_step_change(STEPMOTOR_STRUCT *stepmotor, int step_div, float step_time)
{
	stepmotor_set_step_num(stepmotor, stepmotor->step_count + step_div, step_time);
}

void stepmotor_step_handle_it_update(STEPMOTOR_STRUCT *stepmotor)
{
	stepmotor_stop(stepmotor);
	if(stepmotor->step_count != stepmotor->step_count_set)
	{
		if(stepmotor->step_count > stepmotor->step_count_set)
		{
				stepmotor_decrease_step(stepmotor);
		}
		else
		{
				stepmotor_increase_step(stepmotor);
		}
	}
	else
	{
		stepmotor_step_stop_it(stepmotor);
	}
}

void stepmotor_init(void)
{
	
	TIM1->PSC = 0;
	TIM1->ARR = SystemCoreClock/PWM_MOTOR_FREQ-1;
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 0;
	
	TIM3->PSC = SystemCoreClock/1000000 * TIM_COUNTER_TIME - 1;
	TIM4->PSC = SystemCoreClock/1000000 * TIM_COUNTER_TIME - 1;
	
	float div_angle = 2*PI/STEP_NUMBER;
	float angle = 0;
	for(int i=0; i< STEP_NUMBER; i++)
	{
		sinValue[i] = TIM1->ARR*sin(angle);
		cosValue[i] = TIM1->ARR*cos(angle);
		if(sinValue[i] < 0) sinValue[i] = -sinValue[i];
		if(cosValue[i] < 0) cosValue[i] = -cosValue[i];
		angle += div_angle;
	}
	
	stepmotor1.id = 1;
	stepmotor1.timer = TIM3;
	stepmotor1.port = STEPMOTOR1_PORT;
	stepmotor1.pin[PIN_A2] = STEPMOTOR1_AL_PIN;
	stepmotor1.pin[PIN_B2] = STEPMOTOR1_BL_PIN;
	stepmotor1.pwm_tim = TIM1;
	stepmotor1.pwm_channel[PWM_A1] = TIM_CHANNEL_2;
	stepmotor1.pwm_channel[PWM_B1] = TIM_CHANNEL_1;
	stepmotor1.step_pitch = MICRO_STEP_PITCH;
	
	stepmotor2.id = 2;
	stepmotor2.timer = TIM4;
	stepmotor2.port = STEPMOTOR1_PORT;
	stepmotor2.pin[PIN_A2] = STEPMOTOR2_AL_PIN;
	stepmotor2.pin[PIN_B2] = STEPMOTOR2_BL_PIN;
	stepmotor2.pwm_tim = TIM1;
	stepmotor2.pwm_channel[PWM_A1] = PWM2_AH_CHANNEL;
	stepmotor2.pwm_channel[PWM_B1] = PWM2_BH_CHANNEL;
	stepmotor2.step_pitch = MICRO_STEP_PITCH;
	
	stepmotor1.step_count = 0;
	stepmotor1.step_count_set = 0;
	stepmotor1.status = MOTOR_STOP;
	
	stepmotor2.step_count = 0;
	stepmotor2.step_count_set = 0;
	stepmotor2.status = MOTOR_STOP;
	
	stepmotor_set_step(&stepmotor1, 0);
	stepmotor_set_step(&stepmotor2, 0);
	HAL_Delay(10);
	stepmotor_stop(&stepmotor1); 
	stepmotor_stop(&stepmotor2);
	//test
	HAL_Delay(500);
	stepmotor_move_xy(MOVE_LENGTH_MAX,MOVE_LENGTH_MAX);
	HAL_Delay(500);
	stepmotor_move_xy(0,0);
}


void stepmotor_move_xy(float x, float y)
{
	int step1_num = x/stepmotor1.step_pitch;
	int step2_num = y/stepmotor2.step_pitch;
	
	float step1_diff = step1_num - stepmotor1.step_count;
	float step2_diff = step2_num - stepmotor2.step_count;
	float step_time1 = step_interval, step_time2 = step_interval;
	
	step1_diff = step1_diff<0?-step1_diff:step1_diff;
	step2_diff = step2_diff<0?-step2_diff:step2_diff;
	
	if(step1_diff < step2_diff)
	{
		step_time2 = STEP_TIME_MIN;
		if(step1_diff != 0)
		{			
			step_time1 = 1.0*step_time2*step2_diff/step1_diff;
		}
	}
	else
	{
		step_time1 = STEP_TIME_MIN;
		if(step2_diff != 0)
		{			
			step_time2 = 1.0*step_time1*step1_diff/step2_diff;
		}
	}
	stepmotor_set_step_num(&stepmotor1, step1_num, step_time1);
	stepmotor_set_step_num(&stepmotor2, step2_num, step_time2);
	while(stepmotor1.status == MOTOR_RUN || stepmotor2.status == MOTOR_RUN ){};
}

