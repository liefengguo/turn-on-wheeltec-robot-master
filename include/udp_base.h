
#ifndef __UDP_BASE_H_
#define __UDP_BASE_H_

#include "udp_base.h"
#include <netinet/in.h>
#include <arpa/inet.h>
using namespace std;

//Macro definition
//宏定义
#define SEND_DATA_CHECK   1          //Send data check flag bits //发送数据校验标志位
#define READ_DATA_CHECK   0          //Receive data to check flag bits //接收数据校验标志位
#define FRAME_HEADER      0X7B       //Frame head //帧头
#define FRAME_HEADER_SEC  0X01       //帧头第二位置 父指令码
#define FRAME_HEADER_MAIN_CLEAN  0X02       //帧头第二位置 主刷
#define FRAME_HEADER_SUB_CLEAN  0X03       //帧头第二位置 边刷
#define FRAME_HEADER_FUN  0X04       //帧头第二位置 功能开关
#define FRAME_HEADER_TRE  0X00       //帧头第三位置 子指令
#define FRAME_TAIL        0X7D       //Frame tail //帧尾
#define RECEIVE_DATA_SIZE 24         //The length of the data sent by the lower computer //下位机发送过来的数据的长度
#define SEND_DATA_SIZE    11         //The length of data sent by ROS to the lower machine //ROS向下位机发送的数据的长度
#define PI 				  3.1415926f //PI //圆周率


//The structure of the ROS to send data to the down machine
//ROS向下位机发送数据的结构体
typedef struct _SEND_DATA_  
{
	    uint8_t tx[SEND_DATA_SIZE];
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


#endif