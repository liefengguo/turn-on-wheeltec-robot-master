#include"udp_robot.h"
#define UDP_PORT 1234 // 修改为实际的UDP端口
unsigned char udp_robot::Check_Sum(unsigned char Count_Number,unsigned char mode)
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
void udp_robot::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
    short  transition;  //intermediate variable //中间变量
    Send_Data.tx[0]=FRAME_HEADER; //frame head 0x7B //帧头0X7B
    Send_Data.tx[1] = 0; //set aside //预留位
    Send_Data.tx[2] = 0; //set aside //预留位

    //The target velocity of the X-axis of the robot
    //机器人x轴的目标线速度
    transition=0;
    transition = msg->linear.x*1000; //将浮点数放大一千倍，简化传输
    Send_Data.tx[4] = transition;     //取数据的低8位
    Send_Data.tx[3] = transition>>8;  //取数据的高8位

    //The target velocity of the Y-axis of the robot
    //机器人y轴的目标线速度
    transition=0;
    transition = msg->linear.y*1000;
    Send_Data.tx[6] = transition;
    Send_Data.tx[5] = transition>>8;

    //The target angular velocity of the robot's Z axis
    //机器人z轴的目标角速度
    transition=0;
    transition = msg->angular.z*1000;
    Send_Data.tx[8] = transition;
    Send_Data.tx[7] = transition>>8;

    Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //For the BCC check bits, see the Check_Sum function //BCC校验位，规则参见Check_Sum函数
    Send_Data.tx[10]=FRAME_TAIL; //frame tail 0x7D //帧尾0X7D
    // 发送数据包
    ssize_t sentBytes = sendto(udpSocket, Send_Data.tx, sizeof(Send_Data.tx), 0,
        reinterpret_cast<struct sockaddr*>(&robotAddr), sizeof(robotAddr));

    if (sentBytes < 0)
    {
        ROS_ERROR("Failed to send UDP data");
    }
}
udp_robot::udp_robot(){
    memset(&Send_Data, 0, sizeof(Send_Data));
    ros::NodeHandle nh;
    nh.param<std::string>("robotIP", robotIP , "192.168.100.53" );
    nh.param<int>        ("robotPort", robotPort, 8888); 

    Cmd_Vel_Sub = n.subscribe("cmd_vel", 100, &udp_robot::cmdVelCallback, this); 
    ROS_INFO_STREAM("Data ready"); //Prompt message //提示信息

    udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udpSocket < 0)
    {
        ROS_ERROR("Failed to create UDP socket");
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


int main(int argc, char** argv)
{
    ros::init(argc, argv, "udp_command_sender");
    

    udp_robot Robot_Control;

    ros::spin();

    // 关闭UDP socket
    // close(udpSocket);


    return 0;
}
