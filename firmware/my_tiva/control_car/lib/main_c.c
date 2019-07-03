#include "main.h"
#include "my_pwm.h"
#include "my_PID.h"
#include "my_encoder.h"
#include "my_timer.h"
#include "my_imu.h"
#include "my_def.h"

#define Bus80MHz     4


int32_t qeiPosition, x;
float a,b;
// int test_qei(void);
// int test_PWM(void);

int main_c(void)
{
	// Set the clocking to run directly from the crystal.
	// SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	// test_qei();
	// config();
	// my_imer_init();
	init_PWM();
	// mypwm_setpwm(left_motor,24,toi);
	// mypwm_setpwm(right_motor, 40 , toi);
	my_timer_init();
	my_encoder_init();
	#ifdef USE_IMU
	my_ConfigureIMU();
	#endif

	// my_PID_set_vel_left_sp(10);
	// my_PID_set_vel_right_sp(10);

	// while (1)
	// {
	// 	a= my_PID_get_vel_left_PV();
	// 	b = my_PID_get_vel_right_PV();
	// 	// mypwm_setpwm(1,50,1);
	// 	// qeiPosition = my_encoder_get_left_var();
	// 	// x = my_encoder_get_right_var();
	// 	// qeiPosition = QEIPositionGegt(QEI1_BASE);
	// 	// SysCtlDelay(1000);
	// 	//qeiPosition = my_encoder_getvar();
	// 	// mypwm_setpwm(1, 25, 0);
	// 	// SysCtlDelay(SysCtlClockGet()/3);
	// 	// mypwm_setpwm(0, 60, 0);
	// }
	return 0;
	
}

void ahihi(void)
{
	   
  /* 1) configure the system to use RCC2 for advanced features
      such as 400 MHz PLL and non-integer System Clock Divisor */
  SYSCTL_RCC2_R |= SYSCTL_RCC2_USERCC2;
  /* 2) bypass PLL while initializing */
  SYSCTL_RCC2_R |= SYSCTL_RCC2_BYPASS2;
  /* 3) select the crystal value and oscillator source */
  SYSCTL_RCC_R &= ~SYSCTL_RCC_XTAL_M;   	/* clear XTAL field */
  SYSCTL_RCC_R += SYSCTL_RCC_XTAL_16MHZ;	/* configure for 16 MHz crystal */
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_OSCSRC2_M;	/* clear oscillator source field */
  SYSCTL_RCC2_R += SYSCTL_RCC2_OSCSRC2_MO;	/* configure for main oscillator source */
  /* 4) activate PLL by clearing PWRDN */
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_PWRDN2;
  /* 5) set the desired system divider and the system divider least significant bit */
  SYSCTL_RCC2_R |= SYSCTL_RCC2_DIV400;  	/* use 400 MHz PLL */
  SYSCTL_RCC2_R = (SYSCTL_RCC2_R&~0x1FC00000)   /* clear system clock divider field */
                  + (Bus80MHz<<22);      	/* configure for 80 MHz clock */
  /* 6) wait for the PLL to lock by polling PLLLRIS */
  while((SYSCTL_RIS_R&SYSCTL_RIS_PLLLRIS)==0){
      ;
  }
  /* 7) enable use of PLL by clearing BYPASS */
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_BYPASS2;
}
