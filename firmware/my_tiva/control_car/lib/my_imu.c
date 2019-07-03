#include "my_imu.h"

float theta;
char my_char[10];
float my_float;
#ifdef XE_1
float imu_ref[9] = {0, 32, 55, 84, 115, 250, 313, 340, 360};
//                 {0, 45, 90,135, 180, 225, 270, 315, 360};
#endif
#ifdef XE_2
float imu_ref[9] = {0, 25.5, 61, 103, 135, 220, 275, 322, 360};
//                 {0, 45, 90,135, 180, 225, 270, 315, 360};
#endif
// float imu_calib[8] = {-180, -135,-90,-45,0,45,90,135,180};

void UARTIntHandler1(void);
void my_ConfigureIMU(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // Enable UART3
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTConfigSetExpClk(UART1_BASE, 16000000, 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));

    // IntEnable(INT_UART1);
    // UARTIntEnable(UART1_BASE, UART_INT_RX);
    // UARTIntRegister(UART1_BASE, UARTIntHandler1);
    UARTEnable(UART1_BASE);
}

Quaterniond imu_getQuaterniond(void)
{
    unsigned char temp = 255 - (unsigned char) UARTCharGet(UART1_BASE);
    int temp2 = ((int)temp * 360) / 255;
    theta = temp2;
    theta *= D2R;
    theta = correct_yaw(theta);
    // theta = imu_my_calib(theta);
    float cy = cos(theta * 0.5); //cos(yaw * 0.5);
    float sy = sin(theta * 0.5); //sin(yaw * 0.5);
    float cp = 1;                //cos(pitch * 0.5); pitch = 0
    float sp = 0;                //sin(pitch * 0.5); pitch = 0
    float cr = 1;                //cos(roll * 0.5); roll = 0
    float sr = 0;                //sin(roll * 0.5); roll = 0

    Quaterniond q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
    return q;
}

float imu_getTheta(void)
{
    // UARTCharPut(UART1_BASE, (char)(theta * 255 / 360));
    return theta;
}

// void my_debug(void)
// {
//     UARTCharPut(UART1_BASE, (char)(theta * 255 / 360));
// }

void UARTIntHandler1(void)
{
    static int cnt = -1;
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = UARTIntStatus(UART1_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART1_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while (UARTCharsAvail(UART1_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        char temp = UARTCharGet(UART1_BASE);
        // UARTCharPut(UART0_BASE,temp);

        if (cnt > -1)
        {
            my_char[cnt] = temp;
            //UARTCharPut(UART0_BASE,my_char[cnt]);
            cnt++;
        }
        if (temp == '=')
        {
            cnt = 0;
            //UARTprintf("got");
            my_float = 0;
        }
        if (temp == ',')
        {
            //my_char[cnt] = NULL;
            int cnt_temp = 0;
            /*for(cnt_temp = 1; cnt_temp <= cnt ; cnt_temp ++)
				{
					if(my_char[cnt_temp] == '.')
					{
						//my_float += ((float)my_char[cnt_temp+1])/10 + ((float)my_char[cnt_temp]+2)/100;
						//UARTprintf("  %d \n", (int)my_float);
						//break;
					}
					else
					{
						my_float= my_float*10 + my_char[cnt_temp];
						
						UARTCharPut(UART0_BASE,my_char[cnt_temp]);
						UARTprintf("  %d ", (int)my_float);
					}
				}*/
            //if(my_char[0] == '-') my_float = -my_float;
            cnt = -1;
            //UARTCharPut(UART0_BASE,'\n');
            my_float = strtof(my_char, NULL);
            theta = my_float;
            //UARTprintf("%d \n", (int)my_float);
            //UARTprintf("%f \n", a.my_float);
        }

        //UARTprintf("ahihi");
    }
}

float imu_my_calib(float data)
{
    float results;
    data /= D2R;
    if (data < imu_ref[1])
    {
        results = (data - imu_ref[0]) * 45 / (imu_ref[1] - imu_ref[0]) + 0;
    }
    else if (data < imu_ref[2])
    {
        results = (data - imu_ref[1]) * 45 / (imu_ref[2] - imu_ref[1]) + 45;
    }
    else if (data < imu_ref[3])
    {
        results = (data - imu_ref[2]) * 45 / (imu_ref[3] - imu_ref[2]) + 90;
    }
    else if (data < imu_ref[4])
    {
        results = (data - imu_ref[3]) * 45 / (imu_ref[4] - imu_ref[3]) + 135;
    }
    else if (data < imu_ref[5])
    {
        results = (data - imu_ref[4]) * 45 / (imu_ref[5] - imu_ref[4]) + 180;
    }
    else if (data < imu_ref[6])
    {
        results = (data - imu_ref[5]) * 45 / (imu_ref[6] - imu_ref[5]) + 225;
    }
    else if (data < imu_ref[7])
    {
        results = (data - imu_ref[6]) * 45 / (imu_ref[7] - imu_ref[6]) + 270;
    }
    else
    {
        results = (data - imu_ref[7]) * 45 / (imu_ref[8] - imu_ref[7]) + 315;
    }
    return results*D2R;
}