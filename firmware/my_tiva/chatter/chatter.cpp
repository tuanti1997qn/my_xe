#include <stdbool.h>
#include <stdint.h>
extern "C"
// TivaC specific includes
{
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
}
// ROS includes
#include <ros.h>
#include <std_msgs/String.h>

// ROS nodehandle
ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "Hello world!";

int main(void)
{
    // TivaC application specific code
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();
    // TivaC system clock configuration. Set to 80MHz.
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // ROS nodehandle initialization and topic registration
    nh.initNode();
    nh.advertise(chatter);

    while (1)
    {
        // Publish message to be transmitted.
        str_msg.data = hello;
        chatter.publish(&str_msg);

        // Handle all communications and callbacks.
        nh.spinOnce();

        // Delay for a bit.
        nh.getHardware()->delay(100);
    }
}