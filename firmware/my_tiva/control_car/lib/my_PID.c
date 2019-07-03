#include "my_PID.h"
#include "my_timer.h"
#include "my_pwm.h"
#include "my_encoder.h"
#include <math.h>

/*********************************************/
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
/*****************************************************/

#ifdef XE_1
PID_para
    left =
        {
            .Kp = 500,
            .Ki = 80,
            .Kd = 1},
    right =
        {
            .Kp = 500,
            .Ki = 80,
            .Kd = 1};
#endif

#ifdef XE_2
PID_para
    left =
        {
            .Kp = 500,
            .Ki = 80,
            .Kd = 1},
    right =
        {
            .Kp = 500,
            .Ki = 80,
            .Kd = 1};
#endif

float debug;
float my_debug_fnc(void)
{
    return debug;
}

float vel_left, vel_right, vel_left_sp, vel_right_sp;
float temp_linear, temp_angular;
static my_pos pos =
    {
        .x = 0,
        .y = 0,
        .theta = 0};

static float my_PID_process(float *error, float *pre_error, PID_para *para, huong *dir);

void my_PID_set_vel_left_sp(float value)
{
    vel_left_sp = value;
}

void my_PID_set_vel_right_sp(float value)
{
    vel_right_sp = value;
}

float my_PID_get_vel_left_sp(void)
{
    return vel_left_sp;
}

float my_PID_get_vel_right_sp(void)
{
    return vel_right_sp;
}

void my_PID_set_PID_params(select_motor motor, PID_para *para)
{
    if (motor == left_motor)
    {
        left.Kp = para->Kp;
        left.Ki = para->Ki;
        left.Kd = para->Kd;
    }
    else
    {
        right.Kp = para->Kp;
        right.Ki = para->Ki;
        right.Kd = para->Kd;
    }
}

void my_PID_get_PID_params(select_motor motor, PID_para *para)
{
    if (motor == left_motor)
    {
        para->Kp = left.Kp;
        para->Ki = left.Ki;
        para->Kd = left.Kd;
    }
    else
    {
        para->Kp = right.Kp;
        para->Ki = right.Ki;
        para->Kd = right.Kd;
    }
}

float my_PID_get_vel_left_PV(void)
{
    return vel_left;
}

float my_PID_get_vel_right_PV(void)
{
    return vel_right;
}

void my_PID_set_vel(float linear, float angular) // linear : m/s | angular: rad/s
{
    // cong thuc van toc dai = duong kinh * van toc goc / 2 * pi

    // temp_linear = linear;
    // temp_angular = angular;

    my_PID_set_vel_left_sp(linear - D2W * angular);
    my_PID_set_vel_right_sp(linear + D2W * angular);
}

/***************************** get pos value *******************************/
Quaterniond my_pos_get_Quaternion(void) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    float cy = cos(pos.theta * 0.5); //cos(yaw * 0.5);
    float sy = sin(pos.theta * 0.5); //sin(yaw * 0.5);
    float cp = 1;                    //cos(pitch * 0.5); pitch = 0
    float sp = 0;                    //sin(pitch * 0.5); pitch = 0
    float cr = 1;                    //cos(roll * 0.5); roll = 0
    float sr = 0;                    //sin(roll * 0.5); roll = 0

    Quaterniond q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
    return q;
}

my_pos my_pos_get_pos(void)
{
    return pos;
}

void my_pos_set_pos(my_pos update_pos)
{
    pos = update_pos;
}

void my_pos_set_theta(float new_theta)
{
    pos.theta = new_theta;
}

void my_pos_set_theta_fq(Quaterniond q)
{
    float siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
	float cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
	pos.theta = atan2(siny_cosp, cosy_cosp);
}

/**************************************************************************************/

/************************ static function ****************************************/
static float my_PID_process(float *error, float *pre_error, PID_para *para, huong *dir)
{
    float P_part, I_part, D_part;
    P_part = para->Kp * *error;
    I_part = para->Ki * T * (*error + *pre_error) / 2;
    D_part = para->Kd * (*error - *pre_error) * T;
    *pre_error = *error;
    if (P_part + I_part + D_part > 0)
    {
        *dir = toi;
        return P_part + I_part + D_part;
    }
    else
    {
        *dir = lui;
        return -(P_part + I_part + D_part);
    }
}

static void my_odometry(void)
{
    // cong thuc duoc cho o http://moorerobots.com/blog/post/5?fbclid=IwAR1qnJ5xJERBM6K_v51F8yDjFIzCXEAbo71GJ6sNwJ-OquP3gmXfPHQD8L8
    float R, Wc, Rsin_, Rcos_;
    my_pos pos_update, temp;

    if ((vel_left != vel_right) && ((vel_right - vel_left) * (vel_right - vel_left) < 1000000))
    {
        R = (D2W / 2) * ((vel_left + vel_right) / (vel_right - vel_left));
        Wc = (vel_right - vel_left) / D2W;

        Rsin_ = R * sin(pos.theta);
        Rcos_ = R * cos(pos.theta);

        pos_update.x = cos(Wc * T) * Rsin_ + sin(Wc * T) * Rcos_ + pos.x - Rsin_;
        pos_update.y = sin(Wc * T) * Rsin_ - cos(Wc * T) * Rcos_ + pos.y + Rcos_;
        pos_update.theta = pos.theta + Wc * T;

        pos_update.theta = correct_yaw(pos_update.theta);

        pos.x = pos_update.x;
        pos.y = pos_update.y;
        pos.theta = pos_update.theta;
    }
    else // di thang    
    {
        pos.x = pos.x + T * cos(pos.theta) * (vel_right + vel_left) / 2;
        pos.y = pos.y + T * sin(pos.theta) * (vel_right + vel_left) / 2;
    }
}

extern void my_custom_timer_ISR(void)
{
    static float error_left, pre_error_left, error_right, pre_error_right, duty_temp, error_left_smooth, error_right_smooth, pos_left, pos_right, angle;
    huong dir;

    // //tinh van toc xe
    pos_left = (float)my_encoder_get_left_var() * CVB / XMV;
    vel_left = pos_left / T;
    // tinh sai so va set van toc

    error_left = vel_left_sp - vel_left;
    error_left_smooth = error_left_smooth + 0.25 * (error_left - error_left_smooth); // cong thuc a= a + alpha*(b - a) de lam muot lai error
    if (vel_left_sp == 0)
    {
        mypwm_setpwm(left_motor, 0, dir);
        pre_error_left = vel_left_sp;
    }
    else
    {
        duty_temp = my_PID_process(&error_left_smooth, &pre_error_left, &left, &dir);
        mypwm_setpwm(left_motor, duty_temp + OFF_SET, dir);
        // 1.2 la con so than thanh ma minh chem vao xem thu no co chay tot ko
        // mypwm_setpwm(left_motor, 99, toi);
        // debug = my_PID_process(&error_left, &pre_error_left, &left, &dir);
        // mypwm_setpwm(left_motor, 70, toi);
    }
    //tinh van toc xe

    pos_right = (float)my_encoder_get_right_var() * CVB / XMV;
    vel_right = pos_right / T;
    error_right = vel_right_sp - vel_right;
    error_right_smooth = error_right_smooth + 0.25 * (error_right - error_right_smooth);
    if (vel_right_sp == 0)
    {
        mypwm_setpwm(right_motor, 0, dir);
        pre_error_right = vel_left_sp;
    }
    else
    {
        mypwm_setpwm(right_motor, my_PID_process(&error_right_smooth, &pre_error_right, &right, &dir) + OFF_SET , dir); // mypwm_setpwm(right_motor, 0, toi);
        // 1.2 la con so than thanh ma minh chem vao xem thu no co chay tot ko
    }

    my_odometry();

    // update pos to publish tf.
    // warnning: this pos use Euler angles
}

//float PID_controler(  )
