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
int main(int argc, char** argv)
{
    ros::init(argc, argv, "udp_command_sender");
    ros::NodeHandle nh;

    udp_robot Robot_Control;

    // 创建UDP socket
    int udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udpSocket < 0)
    {
        ROS_ERROR("Failed to create UDP socket");
        return -1;
    }

    // 设置下位机的IP地址和端口号
    std::string robotIP = "192.168.1.100"; // 修改为实际的下位机IP地址
    int robotPort = 5678; // 修改为实际的下位机UDP端口

    struct sockaddr_in robotAddr;
    robotAddr.sin_family = AF_INET;
    robotAddr.sin_port = htons(robotPort);
    robotAddr.sin_addr.s_addr = inet_addr(robotIP.c_str());

    // 订阅cmd_vel话题
    ros::Subscriber cmdVelSub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10,
        [&](const geometry_msgs::Twist::ConstPtr& msg)
        {
            // 构建发送数据包
            unsigned char data[10];
            data[0] = 0x7B; // 帧头
            data[1] = 0x01; // 预留位1
            data[2] = 0x00; // 预留位2
            data[3] = static_cast<unsigned char>(msg->linear.x); // x轴目标速度
            data[4] = static_cast<unsigned char>(msg->linear.y); // y轴目标速度
            data[5] = static_cast<unsigned char>(msg->linear.z); // z轴目标速度
            data[6] = static_cast<unsigned char>(msg->angular.x); // 预留位3
            data[7] = static_cast<unsigned char>(msg->angular.y); // 预留位4
            data[8] = static_cast<unsigned char>(msg->angular.z); // 预留位5
            data[9] = 0x7D; // 帧尾

            // 发送数据包
            ssize_t sentBytes = sendto(udpSocket, data, sizeof(data), 0,
                reinterpret_cast<struct sockaddr*>(&robotAddr), sizeof(robotAddr));

            if (sentBytes < 0)
            {
                ROS_ERROR("Failed to send UDP data");
            }
        });

    ros::spin();

    // 关闭UDP socket
    close(udpSocket);

    return 0;
}
