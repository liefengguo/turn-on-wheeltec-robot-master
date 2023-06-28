#include "udp_log.h"
#include"udp_robot.h"

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "car_info");
//     ros::NodeHandle n;    
//     int sockfd;
//     // udp_robot Robot_Control;
//     struct sockaddr_in serverAddr, clientAddr;
//     socklen_t clientAddrLen = sizeof(clientAddr);
//     ChassisParser chassisParser ;
//     turn_on_wheeltec_robot::Speed speed ;
//     ros::Publisher pub_carSpeed = n.advertise<turn_on_wheeltec_robot::Speed>("/fixposition/speed",10);

//     // 创建UDP套接字
//     if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
//         std::cerr << "Failed to create socket!" << std::endl;
//         return -1;
//     }

//     // 设置服务器地址信息
//     serverAddr.sin_family = AF_INET;
//     serverAddr.sin_port = htons(SERVER_PORT);
//     if (inet_pton(AF_INET, SERVER_IP, &(serverAddr.sin_addr)) <= 0) {
//         std::cerr << "Failed to set server address!" << std::endl;
//         return -1;
//     }

//     // 绑定套接字到本地地址和端口
//     if (bind(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == -1) {
//         std::cerr << "Failed to bind socket!" << std::endl;
//         return -1;
//     }

//     std::cout << "UDP server is listening on " << SERVER_IP << ":" << SERVER_PORT << std::endl;



//     // 接收数据
//     ssize_t numBytesReceived;
//     while (ros::ok) {
//         numBytesReceived = recvfrom(sockfd, chassisParser.receivedData, sizeof(chassisParser.receivedData), 0,
//                                     (struct sockaddr *)&clientAddr, &clientAddrLen);

//         if (numBytesReceived == -1) {
//             std::cerr << "Failed to receive data!" << std::endl;
//             break;
//         } else if (numBytesReceived == 0) {
//             std::cout << "Connection closed by the client." << std::endl;
//             break;
//         } else {
//             std::cout << "Received " << numBytesReceived << " bytes of data from "
//                       << inet_ntoa(clientAddr.sin_addr) << ":" << ntohs(clientAddr.sin_port) << std::endl;

//             // 在这里处理接收到的数据
//             // 可以根据需要进行解析、验证或执行其他操作
//             // 例如，打印接收到的数据内容
//             std::cout << "Received data: ";
//             for (size_t i = 0; i < numBytesReceived; ++i) {
//                 std::cout << std::hex << static_cast<int>(chassisParser.receivedData[i]) << " ";
//             }
//             chassisParser.parseChassisData();
//             chassisParser.logChassisData();
//             // chassisParser.closeLog();
//             std::vector<int32_t> speedData;
//             speedData.push_back(chassisParser.chassisData.carSpeed*1000);// 换算成mm/s

//             speed.speeds = speedData;
//             pub_carSpeed.publish(speed);


//             std::cout << std::endl;
//         }
//     }

//     // 关闭套接字
//     close(sockfd);
//     ros::spin();

//     return 0;
// }
