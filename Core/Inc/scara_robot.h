#ifndef SCARA_ROBOT_H
#define SCARA_ROBOT_H
#include "stm32f1xx_hal.h"

#define PI 											(float)3.141592654
#define X1											(float)0
#define Y1											(float)-19
#define X2											(float)0
#define Y2											(float)19
#define R												(float)80 //mm
#define L												(float)80 //mm
#define M 											(float)20	//mm
#define ANGLE_M									(float)PI*2/3

#define STEP_NUM_OFFSET					(int)20

void scara_init(void);
void scara_set_angle(float angle1_new, float angle2_new);
void scara_move_xy(float x, float y);
void scara_draw_line(float x1, float y1, float x2, float y2);
void scara_draw_rect(float x, float y, float h, float w);
void scara_pen_release(void);
void scara_pen_press(void);
void scara_draw_circle(float x, float y, float r);
#endif
