#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial_msgs/serial.h"
#include <serial/serial.h>
#include <std_msgs/Empty.h>

#define sBUFFERSIZE     25
unsigned char s_buffer[sBUFFERSIZE];
unsigned char r_buffer[5];
serial::Serial ser;

void chatterCallback(const serial_msgs::serial::ConstPtr& msg)
{
     ROS_INFO_STREAM("Writing to serial port");
    for(int i=0;i<msg->serial.size();++i)
       {
     ROS_INFO("Writing: [0x%02x]", msg->serial[i]);
     s_buffer[i]=msg->serial[i];
       }
     ser.write(s_buffer,sBUFFERSIZE);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listen1");
    ros::NodeHandle n;
    //订阅read主题（即接受串口数据）

     try
    {
        ser.setPort("/dev/ttyS0");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
    ros::Subscriber sub = n.subscribe("read1", 1000, chatterCallback);

    ros::spin();

    return 0;
}
