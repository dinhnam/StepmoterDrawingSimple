#include "scara_robot.h"
#include "step_motor.h"
#include "servo.h"
#include <math.h>
#include "my_uart.h"

uint8_t pen_status = 0;

float dist(float x1, float y1, float x2, float y2)
{
	return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
}

uint8_t angle_check(float angle1, float angle2)
{
	if(angle1 == NAN) return 0;
	if(angle2 == NAN) return 0;
	float x1 = X1 + R*cos(angle1);
	float y1 = Y1 + R*sin(angle1);
	float x2 = X2 + R*cos(angle2);
	float y2 = Y2 + R*sin(angle2);
	float d = dist(x1,y1,x2,y2);
	if(d >= 2*L) return 0;
	return 1;
}
int step_offset_sum1 = 0;
int step_offset_sum2 = 0;
int step_diff_sum1 = 0;
int step_diff_sum2 = 0;

void scara_set_offset(STEPMOTOR_STRUCT *stepmotor, int step_num_set)
{
	//stepmotor->diff_sum += step_num_set - stepmotor->step_num;
	if(step_num_set > stepmotor->step_num)
	{
		if(stepmotor->rotate_dir == CLOCKWISE)
		{
			stepmotor->step_num -= STEP_NUM_OFFSET;
			//stepmotor->offset_sum -= STEP_NUM_OFFSET;
		}
	}
	else if(step_num_set < stepmotor->step_num)
	{
		if(stepmotor->rotate_dir == ANTI_CLOCKWISE)
		{
			stepmotor->step_num += STEP_NUM_OFFSET;
			//stepmotor->offset_sum += STEP_NUM_OFFSET;
		}
	}
}

void scara_set_angle(float angle1_new, float angle2_new)
{
//	angle1_new =  roundf(angle1_new * 100) / 100;
//	angle2_new =  roundf(angle2_new * 100) / 100;
	
	int step_num_set1 = angle1_new/STEP_ANGLE;
	int step_num_set2 = angle2_new/STEP_ANGLE;
	
	int step_diff1 = step_num_set1 - stepmotor1.step_num;
	int step_diff2 = step_num_set2 - stepmotor2.step_num;
	
	scara_set_offset(&stepmotor1, step_num_set1);
	scara_set_offset(&stepmotor2, step_num_set2);
	
	
	step_diff1 = step_num_set1 - stepmotor1.step_num;
	step_diff2 = step_num_set2 - stepmotor2.step_num;
	
	int step_diff1_abs = step_diff1<0?-step_diff1:step_diff1;
	int step_diff2_abs = step_diff2<0?-step_diff2:step_diff2;
	int step_time1 = STEP_TIME_MIN, step_time2 = STEP_TIME_MIN;
	
	if(step_diff1_abs < step_diff2_abs)
	{
		if(step_diff1_abs != 0)
		{
			step_time2 = STEP_TIME_MIN;
			step_time1 = step_time2*step_diff2_abs/step_diff1_abs;
		}
	}
	else
	{
		if(step_diff2_abs != 0)
		{
			step_time1 = STEP_TIME_MIN;
			step_time2 = step_time1*step_diff1_abs/step_diff2_abs;
		}
	}
	
	stepmotor_set_angle(&stepmotor1, angle1_new, step_time1);
	stepmotor_set_angle(&stepmotor2, angle2_new, step_time2);
	//printf("step_set1:%d, step_set2:%d\n",step_num_set1, step_num_set2);
	while(stepmotor1.status == MOTOR_RUN ||  stepmotor2.status == MOTOR_RUN){};
	
}

void scara_move_xy(float x, float y)
{
	float c = sqrt(L*L + M*M - 2*L*M*cos(ANGLE_M));
	float d = dist(x, y, X1, Y1);
	float a = acos((R*R+d*d-c*c)/(2*R*d));
	float a2 = atan((y-Y1)/(x-X1));
	float angle1 =  a2 - a;
	float Xr = X1 + R*cos(angle1);
	float Yr = Y1 + R*sin(angle1);
	float e1 = atan( (y-Yr)/(x-Xr));
	float e2 = acos( (L*L + c*c - M*M)/(2*L*c));
	float Xm = Xr + L*cos(e1+e2);
	float Ym = Yr + L*sin(e1+e2);
	float d2 = dist(Xm, Ym, X2, Y2);
	float b = acos( (R*R+ d2*d2 - L*L)/(2*R*d2));
	float b2 = atan((Ym-Y2)/(Xm-X2));
	float angle2 = b + b2;
	if( angle_check(angle1,angle2) == 1 )
	{	
		scara_set_angle(angle1*180/PI, angle2*180/PI);
	}
}

void scara_pen_release(void)
{
	if(pen_status == 1)
	{
		for(int i=ANGLE_PEN_PRESS; i <=ANGLE_PEN_RELEASE; i++)
		{
			servo_rotate(&servo, i);
			HAL_Delay(10);
		}
		pen_status = 0;
	}
}
void scara_pen_press(void)
{
	if(pen_status == 0)
	{
		for(int i=ANGLE_PEN_RELEASE; i >= ANGLE_PEN_PRESS; i--)
		{
			servo_rotate(&servo, i);
			HAL_Delay(10);
		}
		pen_status = 1;
	}
}
void scara_draw_line(float x1, float y1, float x2, float y2)
{
	float div_min = 1;
	float x, y, Dx, Dy;
	float x_div, y_div;
	int d = dist(x1,y1,x2,y2)/div_min;
	if(d<1) d = 1;
	Dx = x2-x1;
	Dy = y2-y1;
	x = x1;
	y = y1;
	x_div = 1.0*Dx/d;
	y_div = 1.0*Dy/d;
	scara_move_xy(x, y);
	for(int i=0; i < d; i++)
	{
		x+=x_div;
		y+=y_div;
		scara_move_xy(x,y);
	}
}
void scara_draw_rect(float x, float y, float w, float h)
{
	scara_pen_release();
	scara_draw_line(x, y, x, y+h);
	scara_draw_line(x, y+h, x+w, y+h);
	scara_draw_line(x+w, y+h, x+w, y);
	scara_draw_line(x+w, y, x, y);
	scara_pen_release();
}

void scara_draw_circle(float x, float y, float r)
{
	float x1 = x+ r*cos(0);
	float y1 = y+ r*sin(0);
	float x2;
	float y2;
	float a;
	scara_pen_release();
	scara_move_xy(x1, y1);
	scara_pen_press();
	for(float i=0; i<=360; i+=5)
	{
		a = i*PI/180;
		x1 = x+ r*cos(a);
		y1 = y+ r*sin(a);
		a = (i+5)*PI/180;
		x2 = x+ r*cos(a);
		y2 = y+ r*sin(a);
		scara_draw_line(x1, y1, x2, y2);
	}
	scara_pen_release();
}

void scara_init(void)
{
	servo_init();
	stepmotor_init();
}
