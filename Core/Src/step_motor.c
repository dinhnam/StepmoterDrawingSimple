#include "step_motor.h"
#include <stdio.h>
#include <math.h>
#include "my_pwm.h"

STEPMOTOR_STRUCT stepmotor1, stepmotor2;

void stepmotor_AHL(STEPMOTOR_STRUCT *stepmotor, float pwm_duty)
{
	PWM_SetDuty(TIM1, stepmotor->pwm_channel[0], TIM1->ARR*pwm_duty);
	PWM_Enable(TIM1, stepmotor->pwm_channel[0]);
	HAL_GPIO_WritePin(stepmotor->port, stepmotor->pin[PIN_A2], GPIO_PIN_RESET);
}

void stepmotor_ALH(STEPMOTOR_STRUCT *stepmotor, float pwm_duty)
{
	PWM_SetDuty(TIM1, stepmotor->pwm_channel[0], TIM1->ARR*(1-pwm_duty));
	PWM_Enable(TIM1, stepmotor->pwm_channel[0]);
	HAL_GPIO_WritePin(stepmotor->port, stepmotor->pin[PIN_A2], GPIO_PIN_SET);
}

void stepmotor_AFL(STEPMOTOR_STRUCT *stepmotor)
{
	PWM_Disable(TIM1, stepmotor->pwm_channel[0]);
	HAL_GPIO_WritePin(stepmotor->port, stepmotor->pin[PIN_A2], GPIO_PIN_RESET);
}

void stepmotor_BHL(STEPMOTOR_STRUCT *stepmotor, float pwm_duty)
{
	PWM_SetDuty(TIM1, stepmotor->pwm_channel[1], TIM1->ARR*pwm_duty);
	PWM_Enable(TIM1, stepmotor->pwm_channel[1]);
	HAL_GPIO_WritePin(stepmotor->port, stepmotor->pin[PIN_B2], GPIO_PIN_RESET);
}

void stepmotor_BLH(STEPMOTOR_STRUCT *stepmotor, float pwm_duty)
{
	PWM_SetDuty(TIM1, stepmotor->pwm_channel[1], TIM1->ARR*(1-pwm_duty));
	PWM_Enable(TIM1, stepmotor->pwm_channel[1]);
	HAL_GPIO_WritePin(stepmotor->port, stepmotor->pin[PIN_B2], GPIO_PIN_SET);
}

void stepmotor_BFL(STEPMOTOR_STRUCT *stepmotor)
{
	PWM_Disable(TIM1, stepmotor->pwm_channel[1]);
	HAL_GPIO_WritePin(stepmotor->port, stepmotor->pin[PIN_B2], GPIO_PIN_RESET);
}

void stepmotor_stop(STEPMOTOR_STRUCT *stepmotor)
{
	stepmotor_AHL(stepmotor, 0);
	stepmotor_BHL(stepmotor, 0);
}

void stepmotor_set_step(STEPMOTOR_STRUCT *stepmotor, uint8_t step)
{
	switch(step)
	{
		case STEP1: stepmotor_AHL(stepmotor, 1);
								stepmotor_BLH(stepmotor, 0);
								break;
		case STEP1A: stepmotor_AHL(stepmotor, 1);
								stepmotor_BHL(stepmotor,0.5);
								break;
		case STEP2: stepmotor_AHL(stepmotor,1);
								stepmotor_BHL(stepmotor,1);
								break;
		case STEP2A: stepmotor_AHL(stepmotor,0.5);
								stepmotor_BHL(stepmotor,1);
								break;
		case STEP3: stepmotor_AHL(stepmotor, 0);
								stepmotor_BHL(stepmotor,1);
								break;
		case STEP3A: stepmotor_ALH(stepmotor,0.5);
								stepmotor_BHL(stepmotor,1);
								break;
		case STEP4: stepmotor_ALH(stepmotor,1);
								stepmotor_BHL(stepmotor,1);
								break;
		case STEP4A: stepmotor_ALH(stepmotor,1);
								stepmotor_BHL(stepmotor,0.5);
								break;
		case STEP5: stepmotor_ALH(stepmotor,1);
								stepmotor_BHL(stepmotor, 0);
								break;
		case STEP5A: stepmotor_ALH(stepmotor,1);
								stepmotor_BLH(stepmotor,0.5);
								break;						
		case STEP6: stepmotor_ALH(stepmotor,1);
								stepmotor_BLH(stepmotor,1);
								break;
		case STEP6A: stepmotor_ALH(stepmotor,0.5);
								stepmotor_BLH(stepmotor,1);
								break;
		case STEP7: stepmotor_ALH(stepmotor, 0);
								stepmotor_BLH(stepmotor,1);
								break;
		case STEP7A: stepmotor_AHL(stepmotor,0.5);
								stepmotor_BLH(stepmotor,1);
								break;
		case STEP8: stepmotor_AHL(stepmotor,1);
								stepmotor_BLH(stepmotor,1);
								break;
		case STEP8A: stepmotor_AHL(stepmotor,1);
								stepmotor_BLH(stepmotor,0.5);
								break;
		default: 	stepmotor_AFL(stepmotor);
							stepmotor_BFL(stepmotor);
							break;
	}
	stepmotor->step = step;
}
void stepmotor_increase_step(STEPMOTOR_STRUCT *stepmotor)
{
	stepmotor->step++;
	if(stepmotor->step >= STEPNUM) stepmotor->step = STEP1;
	stepmotor->step_count++;
	stepmotor_set_step(stepmotor, stepmotor->step);
}
void stepmotor_decrease_step(STEPMOTOR_STRUCT *stepmotor)
{
	stepmotor->step--;
	if(stepmotor->step < STEP1) stepmotor->step = STEPNUM-1;
	stepmotor->step_count--;
	stepmotor_set_step(stepmotor, stepmotor->step);
}

void stepmotor_set_step_time(STEPMOTOR_STRUCT *stepmotor, float step_time_ms)
{ 
	if( step_time_ms < STEP_TIME_MIN)  step_time_ms = STEP_TIME_MIN;
	stepmotor->timer->ARR = step_time_ms*1000/TIM_COUNTER_TIME;
	stepmotor->timer->CCR1 = stepmotor->timer->ARR - STEP_ON_TIME_MIN*1000/TIM_COUNTER_TIME;
}

void stepmotor_step_start_it(STEPMOTOR_STRUCT *stepmotor)
{
	stepmotor->timer->CNT = 0;
	stepmotor->timer->DIER |= TIM_IT_UPDATE;
	stepmotor->timer->DIER |= TIM_IT_CC1;
	stepmotor->timer->CR1 |= TIM_CR1_CEN;
	stepmotor->status = MOTOR_RUN;
}

void stepmotor_step_stop_it(STEPMOTOR_STRUCT *stepmotor)
{
	stepmotor->timer->DIER &= ~TIM_IT_UPDATE;
	stepmotor->timer->DIER &= ~TIM_IT_CC1;
	stepmotor->timer->CR1 &= ~TIM_CR1_CEN;
	stepmotor->status = MOTOR_STOP;
}

void stepmotor_set_step_num(STEPMOTOR_STRUCT *stepmotor, int step_count_set, float step_time)
{
	if( step_count_set > STEP_NUM_MAX) step_count_set = STEP_NUM_MAX;
	if( step_count_set < STEP_NUM_MIN) step_count_set = STEP_NUM_MIN;
	stepmotor->step_count_set = step_count_set;
	stepmotor_set_step_time(stepmotor, step_time);
	if(stepmotor->step_count_set != stepmotor->step_count)
	{
		stepmotor_step_start_it(stepmotor);
	}
}

void stepmotor_set_step_change(STEPMOTOR_STRUCT *stepmotor, int step_div, float step_time)
{
	stepmotor_set_step_num(stepmotor, stepmotor->step_count + step_div, step_time);
}

void stepmotor_step_handle_it_update(STEPMOTOR_STRUCT *stepmotor)
{
	stepmotor_stop(stepmotor);
	if(stepmotor->step_count == stepmotor->step_count_set)
	{
		stepmotor_step_stop_it(stepmotor);
	}
}

void stepmotor_step_handle_it_cc(STEPMOTOR_STRUCT *stepmotor)
{
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
}

void stepmotor_init(void)
{
	TIM1->PSC = 0;
	TIM1->ARR = SystemCoreClock/PWM_STEP_FREQ-1;
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 0;
	
	TIM3->PSC = SystemCoreClock/1000000 * TIM_COUNTER_TIME - 1;
	TIM4->PSC = SystemCoreClock/1000000 * TIM_COUNTER_TIME - 1;
	
	stepmotor1.id = 1;
	stepmotor1.timer = TIM3;
	stepmotor1.port = STEPMOTOR1_PORT;
	stepmotor1.pin[PIN_A2] = STEPMOTOR1_AL_PIN;
	stepmotor1.pin[PIN_B2] = STEPMOTOR1_BL_PIN;
	stepmotor1.pwm_tim = TIM1;
	stepmotor1.pwm_channel[PWM_A1] = TIM_CHANNEL_2;
	stepmotor1.pwm_channel[PWM_B1] = TIM_CHANNEL_1;
	
	
	stepmotor2.id = 2;
	stepmotor2.timer = TIM4;
	stepmotor2.port = STEPMOTOR1_PORT;
	stepmotor2.pin[PIN_B2] = STEPMOTOR2_AL_PIN;
	stepmotor2.pin[PIN_A2] = STEPMOTOR2_BL_PIN;
	stepmotor2.pwm_tim = TIM1;
	stepmotor2.pwm_channel[PWM_B1] = TIM_CHANNEL_4;
	stepmotor2.pwm_channel[PWM_A1] = TIM_CHANNEL_3;
	
	stepmotor1.step_count = 0;
	stepmotor1.step_count_set = 0;
	stepmotor1.status = MOTOR_STOP;
	
	stepmotor2.step_count = 0;
	stepmotor2.step_count_set = 0;
	stepmotor2.status = MOTOR_STOP;
	
	for(int i=1; i<STEPNUM; i++)
	{
		stepmotor_set_step(&stepmotor1, i);
		stepmotor_set_step(&stepmotor2, i);
		HAL_Delay(10);
	}
	
	
	for(int i=STEPNUM-1; i>=1; i--)
	{
		stepmotor_set_step(&stepmotor1, i);
		stepmotor_set_step(&stepmotor2, i);
		HAL_Delay(10);
	}
	//test
	stepmotor_stop(&stepmotor1); 
	stepmotor_stop(&stepmotor2);
	HAL_Delay(500);
	stepmotor_set_step_num(&stepmotor1, STEP_NUM_MAX, STEP_TIME_MIN);
	stepmotor_set_step_num(&stepmotor2, STEP_NUM_MAX, STEP_TIME_MIN);
	while(stepmotor1.status == MOTOR_RUN || stepmotor2.status == MOTOR_RUN ){};
	stepmotor_set_step_num(&stepmotor1, STEP_NUM_MIN, STEP_TIME_MIN);
	stepmotor_set_step_num(&stepmotor2, STEP_NUM_MIN, STEP_TIME_MIN);
	while(stepmotor1.status == MOTOR_RUN || stepmotor2.status == MOTOR_RUN ){};
}


void stepmotor_move_xy(float x, float y)
{
	int step1_num = x/STEP_DIV;
	int step2_num = y/STEP_DIV;
	
	float step1_diff = step1_num - stepmotor1.step_count;
	float step2_diff = step2_num - stepmotor2.step_count;
	float step_time1 = STEP_TIME_MIN, step_time2 = STEP_TIME_MIN;
	
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

float dist(float x1, float y1, float x2, float y2)
{
	return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
}

void stepmotor_move_line(float x1, float y1, float x2, float y2)
{
	float x, y, Dx, Dy;
	float x_div, y_div;
	int d = dist(x1,y1,x2,y2);
	Dx = x2-x1;
	Dy = y2-y1;
	x = x1;
	y = y1;
	x_div = 1.0*Dx/d;
	y_div = 1.0*Dy/d;
	stepmotor_move_xy(x, y);
	for(int i=0; i < d; i++)
	{
		x+=x_div;
		y+=y_div;
		stepmotor_move_xy(x,y);
	}
}
