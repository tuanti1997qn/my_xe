//*****************************************************************************
//
// qei.c - Example to demonstrate QEI on Tiva Launchpad
//This setup uses QEI0 P6/PD7, in my testing an arcade trackball is connected.
//You can also use QEI1 PC5/PC6 in which case you don't need the PD7 HWREG calls (note: I didn't test this)
//
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/qei.h"

#include "my_encoder.h"

int64_t pos;
static double Velocity_left = 0, Velocity_right = 0;



/**************************************************
    PWM chanel 1: ben trai
    QEI0, PWM chanel 2: ben phai
    qei0: ben trai
    QEI1: ben phai
**************************************************/
void my_encoder_init(void)
{

    // Set the clocking to run directly from the crystal.
    // SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Enable QEI Peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);

    //Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    //Set Pins to be PHA0 and PHB0
    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);
    
    /**********************************************************************************************************************/
        // config gpio pin PD0 PD1 de chua bug phan cung minh tao ra
    // GPIOPinTypeGPIOInput(GPIO_PORTD_BASE , GPIO_PIN_0);
    // GPIOPinTypeGPIOInput(GPIO_PORTD_BASE , GPIO_PIN_1);


    /**********************************************************************************************************************/

    //Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7. I believe this sets the pull up and makes them inputs
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    //DISable peripheral and int before configuration
    QEIDisable(QEI0_BASE);
    QEIIntDisable(QEI0_BASE, QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

    // Configure quadrature encoder, use an arbitrary top limit of 1000
    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 400000);

    // Enable the quadrature encoder.
    QEIEnable(QEI0_BASE);

    //Set position to a middle value so we can see if things are working
    QEIPositionSet(QEI0_BASE, 0);

    //Add qeiPosition as a watch expression to see the value inc/dec



    /********************************************************************************************************************************/

        // Set the clocking to run directly from the crystal.
    // SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Enable QEI Peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);

    //Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
    // HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
    // HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    // HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    //Set Pins to be PHA0 and PHB0
    GPIOPinConfigure(GPIO_PC5_PHA1);
    GPIOPinConfigure(GPIO_PC6_PHB1);

    //Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7. I believe this sets the pull up and makes them inputs
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);

    //DISable peripheral and int before configuration
    QEIDisable(QEI1_BASE);
    QEIIntDisable(QEI1_BASE, QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

    // Configure quadrature encoder, use an arbitrary top limit of 1000
    QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 400000);

    // Enable the quadrature encoder.
    QEIEnable(QEI1_BASE);

    //Set position to a middle value so we can see if things are working
    QEIPositionSet(QEI1_BASE, 0);

}

int32_t my_encoder_get_left_var(void)
{
    int32_t left = (int32_t)QEIPositionGet(QEI0_BASE)- 200000; // banh trai la qei0

    // if (left >= 200000)
    //     left -= 400000; // chay nguoc ve am

    QEIPositionSet(QEI0_BASE, 200000);
    return left;
}

int32_t my_encoder_get_right_var(void)
{
    int32_t right = (int32_t)QEIPositionGet(QEI1_BASE) - 200000; // banh phai la qei1

    // if (right >= 200000)
    //     right -= 400000; // chay nguoc ve am

    QEIPositionSet(QEI1_BASE, 200000);
    return -right; // 1 cai xuoi, cai chay nguoc nen cai nay can lam nguoc lai
}

