#include "servo.h"
#include "my_pwm.h"

SERVO_STRUCT servo;

void servo_rotate(SERVO_STRUCT *servo, float roate_angle)
{
	servo->anlge = roate_angle;
	float angle = servo->anlge + servo->angle_offset;
	int pusle_width = 1000 + 1000.0/90 * (angle +45);
	PWM_SetDuty(servo->pwm_tim, servo->tim_channel, pusle_width);
}

void servo_init(void)
{
	TIM2->PSC = SystemCoreClock/1000000-1;
	servo.pwm_tim = SERVO_PWM_TIM;
	servo.tim_channel = SERVO_PWM_CHANNEL;
	servo.angle_offset = SERVO_ANGLE_OFFSET;
	servo.anlge = ANGLE_RELEASE;
	PWM_SetFrequency(servo.pwm_tim, 50);
	PWM_Enable(servo.pwm_tim, servo.tim_channel);
	servo_rotate(&servo, servo.anlge);
}

void servo_pen_release(void)
{
	if(servo.anlge > ANGLE_RELEASE)
	{
//		servo_rotate(&servo, ANGLE_RELEASE);
//		HAL_Delay(40);
		for(servo.anlge = ANGLE_PRESS; servo.anlge>=  ANGLE_RELEASE; servo.anlge--)
		{
			servo_rotate(&servo, servo.anlge);
			HAL_Delay(3);
		}
	}
}

void servo_pen_press(void)
{
	if(servo.anlge < ANGLE_PRESS)
	{
//		servo_rotate(&servo, ANGLE_PRESS);
//		HAL_Delay(40);
		for(servo.anlge = ANGLE_RELEASE; servo.anlge <=  ANGLE_PRESS; servo.anlge++)
		{
			servo_rotate(&servo, servo.anlge);
			HAL_Delay(3);
		}
	}
}
