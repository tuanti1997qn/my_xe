#include "my_math.h"


Quaterniond my_Y2Q(float yaw) // yaw to quaternion
{
    // Abbreviations for the various angular functions
    float cy = cos(yaw * 0.5); //cos(yaw * 0.5);
    float sy = sin(yaw * 0.5); //sin(yaw * 0.5);
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
float my_Q2Y( Quaterniond q ) // quaternion to yaw(rad)
{
    float yaw;
    // // roll (x-axis rotation)
	// float sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
	// float cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	// roll = atan2(sinr_cosp, cosr_cosp);

	// // pitch (y-axis rotation)
	// float sinp = +2.0 * (q.w * q.y - q.z * q.x);
	// if (fabs(sinp) >= 1)
	// 	pitch = copysign(MY_PI / 2, sinp); // use 90 degrees if out of range
	// else
	// 	pitch = asin(sinp);

	// yaw (z-axis rotation)
	float siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
	float cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
	yaw = atan2(siny_cosp, cosy_cosp);
    return yaw;
}

float correct_yaw(float yaw)
{
    if (yaw < 0)
    {
        yaw += 2*MY_PI;
    }
    if (yaw > 2*MY_PI)
    {
        yaw -= 2*MY_PI;
    }
    return yaw;
}