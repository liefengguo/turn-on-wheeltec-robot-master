
#ifndef __UDP_ROBOT_H_
#define __UDP_ROBOT_H_

#include "udp_base.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
//The robot chassis class uses constructors to initialize data, publish topics, etc
//机器人底盘类，使用构造函数初始化数据和发布话题等
class udp_robot
{
	public:
		udp_robot();  
		~udp_robot(); 
		void Control();   //Loop control code //循环控制代码
		
	private:
		ros::NodeHandle n;           //Create a ROS node handle //创建ROS节点句柄

		ros::Subscriber Cmd_Vel_Sub; //Initialize the topic subscriber //初始化话题订阅者
		//The speed topic subscribes to the callback function
		//速度话题订阅回调函数
		void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
		void cmd_main_clean(const std_msgs::Int32::ConstPtr& msg);
		void cmd_sub_clean(const std_msgs::Int32::ConstPtr& msg); 
		void cmd_turn_fun(const std_msgs::Int32::ConstPtr& msg);

        bool Get_Sensor_Data();   
        unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode); //BBC check function //BBC校验函数

        RECEIVE_DATA Receive_Data; //The serial port receives the data structure //串口接收数据结构体
        SEND_DATA Send_Data;       //The serial port sends the data structure //串口发送数据结构体
        std::string robotIP;           //udp IP
        int robotPort;
        int udpSocket;
        struct sockaddr_in robotAddr;

};
#endif
