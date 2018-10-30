#include <ros/ros.h>
#include <string.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "serial_msgs/serial.h"
#include <geometry_msgs/Twist.h>

#define SIZE  16
unsigned char r_buffer[SIZE] = {0};
unsigned char s_buffer[SIZE] = {0};
const unsigned char FW[] = {0xAA,0x7F,0x01,0x01,0x01,0x03,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x55,0x97,0x65};//复位
const unsigned char ZD[] = {0xAA,0x7F,0x01,0x00,0x01,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x55,0x92,0x48};//制动
const unsigned char ZY[] = {0xAA,0x7F,0x01,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x55,0xC7,0x7C};//自由
const unsigned char QJ[] = {0xAA,0x7F,0x01,0x00,0x01,0x01,0x01,0x00,0x01,0x02,0x58,0x02,0x58,0x55,0xFB,0x80};//前进
const unsigned char JS[] = {0xAA,0x7F,0x01,0x00,0x01,0x01,0x01,0x00,0x01,0x03,0xE8,0x03,0xE8,0x55,0xC4,0x80};//加速
const unsigned char HT[] = {0xAA,0x7F,0x01,0x00,0x01,0x01,0x01,0x01,0x00,0x02,0x58,0x02,0x58,0x55,0xEA,0x91};//后退
const unsigned char ZZ[] = {0xAA,0x7F,0x01,0x00,0x01,0x01,0x01,0x01,0x01,0x02,0x58,0x02,0x58,0x55,0xEB,0x40};//左转
const unsigned char YZ[] = {0xAA,0x7F,0x01,0x00,0x01,0x01,0x01,0x00,0x00,0x02,0x58,0x02,0x58,0x55,0xFA,0x51};//右转
const unsigned char header[7] = {0xAA,0x7F,0x01,0x00,0x01,0x01,0x01};
const unsigned char ender = 0x55;

const double ROBOT_RADIUS = 100.00; //轮距半径 mm
const double ROBOT_LENGTH = 210.50;

serial::Serial ser;
double velocity = 0;  // 前进速度 mm/s
double YawRate  = 0;  // 转向速度 rad/s
bool continuous = false;

union send_vel_data
{
    int d;
    unsigned char data[2];
}left_vel, right_vel;

union send_CRC_data
{
    int d;
    unsigned char data[2];
}CRC_16;

unsigned int CheckCRC(unsigned char *buffer,unsigned int buflen)
{
	if (NULL == buffer || buflen <= 0)
	{
		return 0;
	}
	
	unsigned int u16CRC = 0xFFFF;
	for (int i = 0; i < buflen; i++)
	{
		u16CRC ^= (unsigned int)(buffer[i]);
		for(int j = 0; j < 8; j++)
		{
			if (u16CRC & 0x0001)
			{
				u16CRC = (u16CRC >> 1) ^ 0xA001;
			}
			else
			{
				u16CRC = u16CRC >> 1;  
			}
		}
	}
	return u16CRC;
}

void writeSpeed(double RobotV, double RobotY)
{
    int i = 0;
    s_buffer[SIZE] = {0};
// 设置消息头
    for(i = 0; i < 7; i++)
        s_buffer[i] = header[i];

// 设置运动状态  
    if(RobotY > 0)
    {
        puts("turn left");
        s_buffer[7] = 0x01;
        s_buffer[8] = 0x01;
 //       memcpy(s_buffer,ZZ,SIZE);
    }
    else if(RobotY < 0)
    {
        puts("trun right");
        s_buffer[7] = 0x00;
        s_buffer[8] = 0x00;
 //       memcpy(s_buffer,YZ,SIZE);
    }
    else if(RobotV > 0)
    {
        puts("move on");
        s_buffer[7] = 0x00;
        s_buffer[8] = 0x01;
//        memcpy(s_buffer,QJ,SIZE);
    }
    else if(RobotV < 0)
    {
        puts("move back");
        s_buffer[7] = 0x01;
        s_buffer[8] = 0x00;
//        memcpy(s_buffer,HT,SIZE);
    }
    else{
        puts("stop");
        s_buffer[5] = 0x03;
        s_buffer[6] = 0x03;
//        memcpy(s_buffer,ZD,SIZE);
    }
    
// 计算左右轮期望速度
    left_vel.d  = abs(RobotV - RobotY * ROBOT_RADIUS);
    right_vel.d = abs(RobotV + RobotY * ROBOT_RADIUS);
// 设置机器人左右轮速度(高位在前)
    s_buffer[9]  = left_vel.data[1];
    s_buffer[10] = left_vel.data[0];
    s_buffer[11] = right_vel.data[1];
    s_buffer[12] = right_vel.data[0];

// 设置消息尾、校验值（低位在前）
    s_buffer[13] = ender;
    CRC_16.d = CheckCRC(s_buffer,14);
    s_buffer[14] = CRC_16.data[0];
    s_buffer[15] = CRC_16.data[1];
   
    ser.write(s_buffer,SIZE);
//    ROS_INFO("Send:[ %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X ]",s_buffer[0],s_buffer[1],s_buffer[2],s_buffer[3],s_buffer[4],s_buffer[5],s_buffer[6],s_buffer[7],s_buffer[8],s_buffer[9],s_buffer[10],s_buffer[11],s_buffer[12],s_buffer[13],s_buffer[14],s_buffer[15]);
}

void callback(const geometry_msgs::Twist& cmd_vel)
{
    velocity = cmd_vel.linear.x * 1000;
	YawRate = cmd_vel.angular.z;
//    ROS_INFO("Received:[%f,%f]",velocity,YawRate);

    if(!continuous)
    {   //第一次开机先复位
        puts("OK Start");
        ser.write(FW,SIZE);
        continuous = true;
    }
}

int main (int argc, char* argv[])
{
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

    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        writeSpeed(velocity, YawRate);

        if(ser.available())
        {
            ser.read(r_buffer,SIZE);
//            ROS_INFO("Read:[ %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X ]",r_buffer[0],r_buffer[1],r_buffer[2],r_buffer[3],r_buffer[4],r_buffer[5],r_buffer[6],r_buffer[7],r_buffer[8],r_buffer[9],r_buffer[10],r_buffer[11],r_buffer[12],r_buffer[13],r_buffer[14],r_buffer[15]);
        }

        ros::spinOnce();
        loop_rate.sleep();        
    }
}