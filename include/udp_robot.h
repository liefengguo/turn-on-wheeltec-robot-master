
#ifndef __UDP_ROBOT_H_
#define __UDP_ROBOT_H_


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <netinet/in.h>
#include <arpa/inet.h>
using namespace std;

//Macro definition
//宏定义
#define SEND_DATA_CHECK   1          //Send data check flag bits //发送数据校验标志位
#define READ_DATA_CHECK   0          //Receive data to check flag bits //接收数据校验标志位
#define FRAME_HEADER      0X7B       //Frame head //帧头
#define FRAME_TAIL        0X7D       //Frame tail //帧尾
#define RECEIVE_DATA_SIZE 24         //The length of the data sent by the lower computer //下位机发送过来的数据的长度
#define SEND_DATA_SIZE    11         //The length of data sent by ROS to the lower machine //ROS向下位机发送的数据的长度
#define PI 				  3.1415926f //PI //圆周率


//The structure of the ROS to send data to the down machine
//ROS向下位机发送数据的结构体
typedef struct _SEND_DATA_  
{
	    char tx[SEND_DATA_SIZE];
		float X_speed;	       
		float Y_speed;           
		float Z_speed;         
		unsigned char Frame_Tail; 
}SEND_DATA;

//The structure in which the lower computer sends data to the ROS
//下位机向ROS发送数据的结构体
typedef struct _RECEIVE_DATA_     
{
	    uint8_t rx[RECEIVE_DATA_SIZE];
	    uint8_t Flag_Stop;
		unsigned char Frame_Header;
		float X_speed;  
		float Y_speed;  
		float Z_speed;  
		float Power_Voltage;	
		unsigned char Frame_Tail;
}RECEIVE_DATA;


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
