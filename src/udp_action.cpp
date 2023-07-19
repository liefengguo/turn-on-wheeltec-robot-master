#include "udp_action.h"
unsigned char udp_action::Check_Sum(unsigned char Count_Number,unsigned char mode)
{
  unsigned char check_sum=0,k;
  
  if(mode==0) //Receive data mode //接收数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^Receive_Data.rx[k]; //By bit or by bit //按位异或
     }
  }
  if(mode==1) //Send data mode //发送数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^Send_Data.tx[k]; //By bit or by bit //按位异或
     }
  }
  return check_sum; //Returns the bitwise XOR result //返回按位异或结果
}

udp_action::udp_action(){
    memset(&Send_Data, 0, sizeof(Send_Data));
    robotPort = 8888;
    robotIP = "192.168.100.53";


    udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udpSocket < 0)
    {
        printf("Failed to create UDP socket");
    }
    try
    {
        robotAddr.sin_family = AF_INET;
        robotAddr.sin_port = htons(robotPort);
        robotAddr.sin_addr.s_addr = inet_addr(robotIP.c_str());
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}
udp_action::~udp_action(){
        //对象turn_on_robot结束前向下位机发送停止运动命令
    Send_Data.tx[0]=FRAME_HEADER;
    Send_Data.tx[1] = FRAME_HEADER_SEC;  
    Send_Data.tx[2] = FRAME_HEADER_TRE; 

    //The target velocity of the X-axis of the robot //机器人X轴的目标线速度 
    Send_Data.tx[4] = 0;     
    Send_Data.tx[3] = 0;  

    //The target velocity of the Y-axis of the robot //机器人Y轴的目标线速度 
    Send_Data.tx[6] = 0;
    Send_Data.tx[5] = 0;  

    //The target velocity of the Z-axis of the robot //机器人Z轴的目标角速度 
    Send_Data.tx[8] = 0;  
    Send_Data.tx[7] = 0;    
    Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //Check the bits for the Check_Sum function //校验位，规则参见Check_Sum函数
    Send_Data.tx[10]=FRAME_TAIL; 
    ssize_t sentBytes = sendto(udpSocket, Send_Data.tx, sizeof(Send_Data.tx), 0,
    reinterpret_cast<struct sockaddr*>(&robotAddr), sizeof(robotAddr));

    close(udpSocket);
}

void udp_action::cmd_main_clean(uint8_t cmd,uint16_t data){
    short  transition;  //intermediate variable //中间变量
    char * send_buf = new char[11];
    Send_Data.tx[0]=FRAME_HEADER; //frame head 0x7B //帧头0X7B
    Send_Data.tx[1] = FRAME_HEADER_MAIN_CLEAN; //set aside //预留位
    Send_Data.tx[2] = cmd; //set aside //预留位
    if(cmd == 1){
        Send_Data.tx[3] = data;
        Send_Data.tx[4] = 0;     
    }else if(cmd == 2){
        transition=0;
        if(data >3000) data = 3000;
        transition = data*1000; //将浮点数放大一千倍，简化传输
        Send_Data.tx[4] = transition;     //取数据的低8位
        Send_Data.tx[3] = transition>>8;  //取数据的高8位
    }

    Send_Data.tx[6] = 0;
    Send_Data.tx[5] = 0;  
    Send_Data.tx[8] = 0;  
    Send_Data.tx[7] = 0;    
    Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //Check the bits for the Check_Sum function //校验位，规则参见Check_Sum函数
    Send_Data.tx[10]=FRAME_TAIL; 
    // 发送数据包
    ssize_t sentBytes = sendto(udpSocket, Send_Data.tx, sizeof(Send_Data.tx), 0,
        reinterpret_cast<struct sockaddr*>(&robotAddr), sizeof(robotAddr));
    std::cout<<">.<!"<<endl;
    // sprintf((char)(Send_Data.tx));
    for (size_t i = 0; i < sizeof(Send_Data.tx); ++i) {
        std::cout << std::hex << static_cast<int>(Send_Data.tx[i])<<"  ";
        // std::cout << std::hex << static_cast<int>(Send_Data.tx[i])<<"  " << i << " "<<endl;
    }
    if (sentBytes < 0)
    {
        printf("Failed to send UDP data");
    }
}

void udp_action::cmd_sub_clean(uint8_t cmd,uint16_t data){
    short  transition;  //intermediate variable //中间变量
    char * send_buf = new char[11];
    Send_Data.tx[0]=FRAME_HEADER; //frame head 0x7B //帧头0X7B
    Send_Data.tx[1] = FRAME_HEADER_SUB_CLEAN; //set aside //预留位
    Send_Data.tx[2] = cmd; //set aside //预留位

    Send_Data.tx[3] = data;

    Send_Data.tx[4] = 0;     

    //The target velocity of the Y-axis of the robot //机器人Y轴的目标线速度 
    Send_Data.tx[6] = 0;
    Send_Data.tx[5] = 0;  

    //The target velocity of the Z-axis of the robot //机器人Z轴的目标角速度 
    Send_Data.tx[8] = 0;  
    Send_Data.tx[7] = 0;    
    Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //Check the bits for the Check_Sum function //校验位，规则参见Check_Sum函数
    Send_Data.tx[10]=FRAME_TAIL; 
    // 发送数据包
    ssize_t sentBytes = sendto(udpSocket, Send_Data.tx, sizeof(Send_Data.tx), 0,
        reinterpret_cast<struct sockaddr*>(&robotAddr), sizeof(robotAddr));
    std::cout<<">.<!"<<endl;
    // sprintf((char)(Send_Data.tx));
    for (size_t i = 0; i < sizeof(Send_Data.tx); ++i) {
        std::cout << std::hex << static_cast<int>(Send_Data.tx[i])<<"  ";
        // std::cout << std::hex << static_cast<int>(Send_Data.tx[i])<<"  " << i << " "<<endl;
    }
    if (sentBytes < 0)
    {
        printf("Failed to send UDP data");
    }
}
void udp_action::cmd_turn_fun(uint8_t cmd , uint16_t data){
    short  transition;  //intermediate variable //中间变量
    char * send_buf = new char[11];
    Send_Data.tx[0]=FRAME_HEADER; //frame head 0x7B //帧头0X7B
    Send_Data.tx[1] = FRAME_HEADER_FUN; //set aside //预留位
    Send_Data.tx[2] = cmd; //set aside //预留位
    Send_Data.tx[3] = data;
    Send_Data.tx[4] = 0;     
    Send_Data.tx[6] = 0;
    Send_Data.tx[5] = 0;  
    Send_Data.tx[8] = 0;  
    Send_Data.tx[7] = 0;    
    Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //Check the bits for the Check_Sum function //校验位，规则参见Check_Sum函数
    Send_Data.tx[10]=FRAME_TAIL; 
    // 发送数据包
    ssize_t sentBytes = sendto(udpSocket, Send_Data.tx, sizeof(Send_Data.tx), 0,
        reinterpret_cast<struct sockaddr*>(&robotAddr), sizeof(robotAddr));
    std::cout<<">.<!"<<endl;
    // sprintf((char)(Send_Data.tx));
    for (size_t i = 0; i < sizeof(Send_Data.tx); ++i) {
        std::cout << std::hex << static_cast<int>(Send_Data.tx[i])<<"  ";
        // std::cout << std::hex << static_cast<int>(Send_Data.tx[i])<<"  " << i << " "<<endl;
    }
    if (sentBytes < 0)
    {
        printf("Failed to send UDP data");
    }
}
void udp_action::cmd_turn_light(uint8_t left , uint8_t right){
    short  transition;  //intermediate variable //中间变量
    char * send_buf = new char[11];
    Send_Data.tx[0]=FRAME_HEADER; //frame head 0x7B //帧头0X7B
    Send_Data.tx[1] = FRAME_HEADER_FUN; //set aside //预留位
    Send_Data.tx[2] = 2; 
    Send_Data.tx[3] = left;
    Send_Data.tx[4] = right;     
    Send_Data.tx[6] = 0;
    Send_Data.tx[5] = 0;  
    Send_Data.tx[8] = 0;  
    Send_Data.tx[7] = 0;    
    Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //Check the bits for the Check_Sum function //校验位，规则参见Check_Sum函数
    Send_Data.tx[10]=FRAME_TAIL; 
    // 发送数据包
    ssize_t sentBytes = sendto(udpSocket, Send_Data.tx, sizeof(Send_Data.tx), 0,
        reinterpret_cast<struct sockaddr*>(&robotAddr), sizeof(robotAddr));
    std::cout<<">.<!"<<endl;
    // sprintf((char)(Send_Data.tx));
    for (size_t i = 0; i < sizeof(Send_Data.tx); ++i) {
        std::cout << std::hex << static_cast<int>(Send_Data.tx[i])<<"  ";
        // std::cout << std::hex << static_cast<int>(Send_Data.tx[i])<<"  " << i << " "<<endl;
    }
    if (sentBytes < 0)
    {
        printf("Failed to send UDP data");
    }
}

void udp_action::main_up(uint8_t up){
    cmd_main_clean(1,up);
}
void udp_action::main_clean(uint16_t data){
    cmd_main_clean(2,data);
}
void udp_action::sub_up(uint8_t up){cmd_sub_clean(1,up);}
void udp_action::sub_out(uint8_t out){cmd_sub_clean(2,out);}
void udp_action::sub_clean(uint8_t up){cmd_sub_clean(3,up);}
void udp_action::big_light(uint8_t up){cmd_turn_fun(1,up);}
void udp_action::left_light(uint8_t up){cmd_turn_light(up,0);}
void udp_action::right_light(uint8_t up){cmd_turn_light(0,up);}
void udp_action::fan(uint8_t up){cmd_turn_fun(3,up);}
void udp_action::pump(uint8_t up){cmd_turn_fun(4,up);}

void udp_action::precursor(uint8_t up){
    short  transition;  //intermediate variable //中间变量
    char * send_buf = new char[11];
    Send_Data.tx[0]=FRAME_HEADER; //frame head 0x7B //帧头0X7B
    Send_Data.tx[1] = 6; //set aside //预留位
    Send_Data.tx[2] = 1; //set aside //预留位

    Send_Data.tx[3] = up;

    Send_Data.tx[4] = 0;     

    //The target velocity of the Y-axis of the robot //机器人Y轴的目标线速度 
    Send_Data.tx[6] = 0;
    Send_Data.tx[5] = 0;  

    //The target velocity of the Z-axis of the robot //机器人Z轴的目标角速度 
    Send_Data.tx[8] = 0;  
    Send_Data.tx[7] = 0;    
    Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //Check the bits for the Check_Sum function //校验位，规则参见Check_Sum函数
    Send_Data.tx[10]=FRAME_TAIL; 
    // 发送数据包
    ssize_t sentBytes = sendto(udpSocket, Send_Data.tx, sizeof(Send_Data.tx), 0,
        reinterpret_cast<struct sockaddr*>(&robotAddr), sizeof(robotAddr));
    std::cout<<">.<!"<<endl;
    // sprintf((char)(Send_Data.tx));
    for (size_t i = 0; i < sizeof(Send_Data.tx); ++i) {
        std::cout << std::hex << static_cast<int>(Send_Data.tx[i])<<"  ";
        // std::cout << std::hex << static_cast<int>(Send_Data.tx[i])<<"  " << i << " "<<endl;
    }
    if (sentBytes < 0)
    {
        printf("Failed to send UDP data");
    }
}

// int main(int argc, char** argv)
// {
//     udp_action robot_action;
//     robot_action.main_clean(20);
//     robot_action.precursor(1);
//     // robot_action.cmd_main_clean(2,30);
//     return 0;
// }
