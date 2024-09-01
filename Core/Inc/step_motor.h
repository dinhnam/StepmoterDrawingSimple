#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H
#include "stm32f1xx_hal.h"

#define PWM_MOTOR_FREQ			(int)10000
#define PI									(float)3.14159265
#define ANGLE_STEP					(float)18 //degree
#define SCREW_PITCH					(float)4 //mm
#define	STEP_NUMBER					(int)64
#define MOVE_LENGTH_MAX			(float)34 //mm
#define STEP_PITCH					(float)(SCREW_PITCH*ANGLE_STEP/360) //mm
#define MICRO_STEP_PITCH		(float)(4*STEP_PITCH/STEP_NUMBER)
#define STEP_NUM_MAX				(int)(MOVE_LENGTH_MAX/MICRO_STEP_PITCH)
#define STEP_NUM_MIN				(int)0
#define TIM_COUNTER_TIME		(int)10 //us
#define STEP_TIME_MIN				(int)1000//us

#define STEPMOTOR1_PORT			GPIOB
#define STEPMOTOR1_AL_PIN		GPIO_PIN_13
#define STEPMOTOR1_BL_PIN		GPIO_PIN_12

#define STEPMOTOR2_PORT			GPIOB
#define STEPMOTOR2_AL_PIN		GPIO_PIN_14
#define STEPMOTOR2_BL_PIN		GPIO_PIN_15

#define PWM1_AH_CHANNEL			TIM_CHANNEL_2
#define PWM1_BH_CHANNEL			TIM_CHANNEL_1

#define PWM2_AH_CHANNEL			TIM_CHANNEL_3
#define PWM2_BH_CHANNEL			TIM_CHANNEL_4
typedef enum
{
	PIN_A2 = 0,
	PIN_B2
}PIN_CTRL_IDX;

typedef enum
{
	PWM_A1 = 0,
	PWM_B1
}PWM_PIN_IDX;

typedef enum
{
	MOTOR_STOP = 0,
	MOTOR_RUN = 1
}STEPMOTOR_STATUS;

typedef struct{
	uint8_t 			id;
	TIM_TypeDef * pwm_tim;
	uint32_t 			pwm_channel[2];
	GPIO_TypeDef *port;
	uint32_t 			pin[2];
		TIM_TypeDef * timer;
	__IO uint8_t 	status;
	int 					step;
	int 					step_count;
	int 					step_count_set;
	float					step_pitch;
}STEPMOTOR_STRUCT;

extern STEPMOTOR_STRUCT stepmotor1, stepmotor2;
extern float step_interval;

void stepmotor_step_stop(STEPMOTOR_STRUCT *stepmotor);
void stepmotor_set_step(STEPMOTOR_STRUCT *stepmotor, int step);
void stepmotor_increase_step(STEPMOTOR_STRUCT *stepmotor);
void stepmotor_decrease_step(STEPMOTOR_STRUCT *stepmotor);
void stepmotor_set_step_time(STEPMOTOR_STRUCT *stepmotor, float step_time_us);
void stepmotor_set_step_change(STEPMOTOR_STRUCT *stepmotor, int step_div, float step_time);
void stepmotor_set_step_num(STEPMOTOR_STRUCT *stepmotor, int step_count_set, float step_time);
void stepmotor_step_start_it(STEPMOTOR_STRUCT *stepmotor);
void stepmotor_step_stop_it(STEPMOTOR_STRUCT *stepmotor);
void stepmotor_step_handle_it_update(STEPMOTOR_STRUCT *stepmotor);
void stepmotor_init(void);
void stepmotor_move_xy(float x, float y);
#endif
