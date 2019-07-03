#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu9150.h"
#include "sensorlib/comp_dcm.h"
#include "my_PID.h"
#include "math.h"
#include "my_def.h"
#include "my_math.h"

#define M_PI 3.14159265358979323846
#define D2R M_PI/180.0 // convert degree to radian 

void my_ConfigureIMU(void);
Quaterniond imu_getQuaterniond(void);
float imu_getTheta(void);
float imu_my_calib(float data);