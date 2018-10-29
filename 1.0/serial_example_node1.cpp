#include <ros/ros.h>
#include <string.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "serial_msgs/serial.h"
#include <geometry_msgs/Twist.h>

#define SIZE  16
unsigned char r_buffer[SIZE];
unsigned char s_buffer[SIZE];
unsigned char FW[] = {0xAA,0x7F,0x01,0x01,0x01,0x03,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x55,0x97,0x65};//复位
unsigned char ZD[] = {0xAA,0x7F,0x01,0x00,0x01,0x03,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x55,0x93,0x99};//制动
unsigned char ZY[] = {0xAA,0x7F,0x01,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x55,0xC7,0x7C};//自由
unsigned char QJ[] = {0xAA,0x7F,0x01,0x00,0x01,0x01,0x01,0x00,0x01,0x02,0x58,0x02,0x58,0x55,0xFB,0x80};//前进
unsigned char JS[] = {0xAA,0x7F,0x01,0x00,0x01,0x01,0x01,0x00,0x01,0x03,0xE8,0x03,0xE8,0x55,0xC4,0x80};//加速
unsigned char HT[] = {0xAA,0x7F,0x01,0x00,0x01,0x01,0x01,0x01,0x00,0x02,0x58,0x02,0x58,0x55,0xEA,0x91};//后退
unsigned char ZZ[] = {0xAA,0x7F,0x01,0x00,0x01,0x01,0x01,0x01,0x01,0x02,0x58,0x02,0x58,0x55,0xEB,0x40};//左转
unsigned char YZ[] = {0xAA,0x7F,0x01,0x00,0x01,0x01,0x01,0x00,0x00,0x02,0x58,0x02,0x58,0x55,0xFA,0x51};//右转

serial::Serial ser;
bool continuous = false;

void callback(const geometry_msgs::Twist& cmd_vel)
{
//    puts("Received a /cmd_vel message!");
//    ROS_INFO("Linear Components:[%f,%f,%f]",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.linear.z);
//    ROS_INFO("Angular Components:[%f,%f,%f]",cmd_vel.angular.x,cmd_vel.angular.y,cmd_vel.angular.z);
    if(!continuous){   //第一次开机先复位
        puts("OK Start");
        ser.write(FW,SIZE);
        continuous = true;
    }

    if(cmd_vel.linear.x>0){//前进
        puts("move on");
        memcpy(s_buffer,QJ,SIZE);
    }
    else if(cmd_vel.linear.x<0){//后退
        puts("move back");
        memcpy(s_buffer,HT,SIZE);
    }
    else if(cmd_vel.angular.z>0){//左转
        puts("turn left");
        memcpy(s_buffer,ZZ,SIZE);
    }
    else if(cmd_vel.angular.z<0){//右转
        puts("trun right");
        memcpy(s_buffer,YZ,SIZE);
    }
    else{
        puts("stop");
        memcpy(s_buffer,ZD,SIZE);//制动
    }

    ser.write(s_buffer,SIZE);
//    puts("Send OK");
}


int main (int argc, char* argv[]){
    ros::init(argc, argv, "serial_example_node1");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("cmd_vel", 50, callback);
    ros::Publisher msg_pub = nh.advertise<serial_msgs::serial>("read1", 1000);

    try
    {
      //串口设置
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
      /*如果dmesg命令能看到串口而打开失败，一般是权限问题，使用命令打开对应的串口，sudo chmod 666 /dev/ttyS0*/
        puts("Unable to open port ");
        return -1;
    }

    if(ser.isOpen())     { puts("Serial Port initialized"); }
    else                 { return -1; }

    //ros::Rate loop_rate(50);

    while(ros::ok()){
        serial_msgs::serial msg;

        ros::Rate loop_rate(50);

        if(ser.available()){
            puts("Read serial port: ");
            ser.read(r_buffer,SIZE);

            for(int i=0;i<SIZE;i++)
            {
                ROS_INFO("0x%02x",r_buffer[i]);
            }

            puts("End reading from serial port");

        }

        ros::spinOnce();
        loop_rate.sleep();        
    }
}
