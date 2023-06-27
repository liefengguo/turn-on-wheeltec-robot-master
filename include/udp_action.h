#ifndef __UDP_ACTION_H_
#define __UDP_ACTION_H_

#include "udp_base.h"
#include <std_msgs/Int32.h>

class udp_action
{
	public:
		udp_action();  
		~udp_action(); 
		// void Control();   //Loop control code //循环控制代码

        void main_up(uint8_t up);
        void main_clean(uint16_t data);
        void sub_up(uint8_t up);
        void sub_out(uint8_t out);
        void sub_clean(uint8_t up);
        void big_light(uint8_t up);
        void left_light(uint8_t up);
        void right_light(uint8_t up);
        void fan(uint8_t up);
        void pump(uint8_t up);
        void precursor(uint8_t up);
		
	private:
        unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode); //BBC check function //BBC校验函数

		void cmd_main_clean(uint8_t cmd,uint16_t data);
		void cmd_sub_clean(uint8_t cmd, uint16_t data); 
		void cmd_turn_fun(uint8_t cmd , uint16_t data);
        void cmd_turn_light(uint8_t left,uint8_t right);
        RECEIVE_DATA Receive_Data; //The serial port receives the data structure //串口接收数据结构体
        SEND_DATA Send_Data;       //The serial port sends the data structure //串口发送数据结构体
        std::string robotIP;           //udp IP
        int robotPort;
        int udpSocket;
        struct sockaddr_in robotAddr;

};
#endif