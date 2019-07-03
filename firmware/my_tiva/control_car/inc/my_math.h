#ifndef __MY_MATH_H_
#define __MY_MATH_H_

#include "math.h"

#define MY_PI 3.1415926535897 // so pi :V
typedef struct
{
    float x;
    float y;
    float z;
    float w;
} Quaterniond;

Quaterniond my_Y2Q(float yaw); // yaw (rad) to quaternion
float my_Q2Y(Quaterniond q);   // quaternion to yaw (rad)
float correct_yaw(float yaw);
#endif /* __MY_MATH_H_*/
