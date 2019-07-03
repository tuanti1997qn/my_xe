#ifndef __MY_PID__
#define __MY_PID__

#include "my_pwm.h"
#include "my_def.h"
#include "my_math.h"
// vi hoi lam bieng nen phan tinhs tf transform se quang vao day luon



#define T 0.01

#ifdef XE_1
#define XMV 1496              // xung moi vong 374*4(do tiva * 4)
#define CVB 0.2             // chu vi banh 6.2 cm
#define D2W 0.19             // distance 2 wheel
#define OFF_SET 5          // offset xung PWM 7% de chong vung chet
#endif

#ifdef XE_2
#define XMV 1496              // xung moi vong 374*4(do tiva * 4)
#define CVB 0.204             // duong kinh banh 65mm
#define D2W 0.197              // distance 2 wheel
#define OFF_SET 7          // offset xung PWM 7% de chong vung chet
#endif

typedef struct
{
    float Kp;
    float Kd;
    float Ki;
} PID_para;

typedef struct
{
    float x;
    float y;
    float z;
} my_pos_linear;

typedef struct
{
    float x;
    float y;
    float z;
} my_pos_angular_euler;



typedef struct
{
    float x;
    float y;
    float theta;
} my_pos;

void my_PID_set_vel_left_sp(float value);
void my_PID_set_vel_right_sp(float value);
float my_PID_get_vel_left_sp(void);
float my_PID_get_vel_right_sp(void);
void my_PID_set_PID_params(select_motor motor, PID_para *para);
void my_PID_get_PID_params(select_motor motor, PID_para *para);
float my_PID_get_vel_left_PV(void);
float my_PID_get_vel_right_PV(void);
void my_PID_set_vel(float linear, float angular);

Quaterniond my_pos_get_Quaternion(void);
my_pos my_pos_get_pos(void);
void my_pos_set_pos(my_pos update_pos);
void my_pos_set_theta(float new_theta);
void my_pos_set_theta_fq(Quaterniond q);

float my_debug_fnc(void);

#endif // __MY_PID__
