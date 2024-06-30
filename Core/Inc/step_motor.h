#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H
#include "stm32f1xx_hal.h"

#define PWM_STEP_FREQ				(int)5000
#define PI									(float)3.14159265
#define STEP_ANGLE					(float)9.0
#define STEP_DIV						(float)0.05 //mm
#define MOVE_LENGTH_MAX			(float)35 //mm
#define STEP_NUM_MAX				MOVE_LENGTH_MAX/STEP_DIV
#define STEP_NUM_MIN				0
#define TIM_COUNTER_TIME		(int)10 //us
#define STEP_TIME_MIN				(float)1.6//ms
#define STEP_ON_TIME_MIN		(float)1.5//ms

#define STEPMOTOR1_PORT			GPIOB
#define STEPMOTOR1_AL_PIN		GPIO_PIN_13
#define STEPMOTOR1_BL_PIN		GPIO_PIN_12

#define STEPMOTOR2_PORT			GPIOB
#define STEPMOTOR2_AL_PIN		GPIO_PIN_15
#define STEPMOTOR2_BL_PIN		GPIO_PIN_14

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
	STEP1 = 1,
	STEP1A,
	STEP2,
	STEP2A,
	STEP3,
	STEP3A,
	STEP4,
	STEP4A,
	STEP5,
	STEP5A,
	STEP6,
	STEP6A,
	STEP7,
	STEP7A,
	STEP8,
	STEP8A,
	STEPNUM
}STEP_IDX;

typedef enum
{
	ANTI_CLOCKWISE = 0,
	CLOCKWISE = 1
}DIRECT_TYPE;

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
	int 					step;
	int 					step_count;
	int 					step_count_set;
	uint8_t				step_active;
	float					angle;
	uint8_t 			rotate_dir;
	TIM_TypeDef * timer;
	__IO uint8_t 	status;
	int 					offset_sum;
	int 					diff_sum;
}STEPMOTOR_STRUCT;

extern STEPMOTOR_STRUCT stepmotor1, stepmotor2;

void stepmotor_step_stop(STEPMOTOR_STRUCT *stepmotor);
void stepmotor_set_step(STEPMOTOR_STRUCT *stepmotor, uint8_t step);
void stepmotor_increase_step(STEPMOTOR_STRUCT *stepmotor);
void stepmotor_decrease_step(STEPMOTOR_STRUCT *stepmotor);
void stepmotor_set_step_time(STEPMOTOR_STRUCT *stepmotor, float step_time_us);
void stepmotor_set_step_change(STEPMOTOR_STRUCT *stepmotor, int step_div, float step_time);
void stepmotor_set_step_num(STEPMOTOR_STRUCT *stepmotor, int step_count_set, float step_time);
void stepmotor_set_rotate_direct(STEPMOTOR_STRUCT *stepmotor, DIRECT_TYPE rotate_direct);
void stepmotor_step_start_it(STEPMOTOR_STRUCT *stepmotor);
void stepmotor_step_stop_it(STEPMOTOR_STRUCT *stepmotor);
void stepmotor_step_handle_it_cc(STEPMOTOR_STRUCT *stepmotor);
void stepmotor_step_handle_it_update(STEPMOTOR_STRUCT *stepmotor);
void stepmotor_init(void);
void stepmotor_move_xy(float x, float y);
void stepmotor_move_line(float x1, float y1, float x2, float y2);
#endif
