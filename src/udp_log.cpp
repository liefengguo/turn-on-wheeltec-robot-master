#include <stdio.h>
#include <time.h>
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include<fstream> 
#include <sstream>

#include <algorithm>
#include <chrono>
#define SERVER_IP "127.0.0.1" // 本地IP地址
#define SERVER_PORT 8888      // 监听端口号
uint8_t receivedData[50];

// 定义包含解析数据的结构体
struct ChassisData {
    uint16_t batSOC;
    uint16_t batV;
    uint16_t batA;
    uint16_t batAlarm;
    uint16_t batTemp1;
    uint16_t batTemp2;
    uint16_t carSpeed;
    int32_t encoderValue;
    int8_t bodyTemperature;
    int8_t bodyHumidity;
    float angleX;
    float angleY;
    float angleZ;
};

class ChassisParser {
public:
    ChassisParser() {
        // 初始化成员变量
        chassisData = ChassisData();
        void closeLog();
        // 生成日志文件名（包含今天的日期）
        std::stringstream filename;
        auto now = std::chrono::system_clock::now();
        std::time_t timestamp = std::chrono::system_clock::to_time_t(now);
        struct tm* timeinfo = std::localtime(&timestamp);
        std::string path = "src/turn-on-wheeltec-robot-master/log/";

        filename <<path<< "chassis_log_" << (timeinfo->tm_year + 1900) << "-"
                 << (timeinfo->tm_mon + 1) << "-" << timeinfo->tm_mday << ".txt";
        std::cout<<"file path :"<<filename.str()<<std::endl;
        

        

        // 打开日志文件
        logfile.open(filename.str(), std::ios::app);

        // 检查文件是否成功打开
        if (!logfile.is_open()) {
            std::cerr << "Failed to open log file: " << filename.str() << std::endl;
        }
    }
    ~ChassisParser() {
        // 在析构函数中关闭日志文件
        if (logfile.is_open()) {
            std::cout<<"close!!!!"<<std::endl;
            logfile.close();
        }
    }
    void closeLog() {
        // 在析构函数中关闭日志文件
        if (logfile.is_open()) {
            std::cout<<"close!!!!"<<std::endl;
            logfile.close();
        }
    }

    void logChassisData() {
        // 检查日志文件是否成功打开
        if (!logfile.is_open()) {
            // 处理无法打开日志文件的情况
            std::cout<<"no open!!!!"<<std::endl;
            return;
        }

        
        // 获取当前时间
        std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
        std::time_t timestamp = std::chrono::system_clock::to_time_t(now);

        // 将时间戳写入日志文件
        logfile << "Timestamp: " << std::ctime(&timestamp);

        // 将chassisData的变量写入日志文件
        logfile << "Battery SOC: " << chassisData.batSOC << "%" << std::endl;
        logfile << "Battery Voltage: " << chassisData.batV << "V" << std::endl;
        logfile << "Battery Current: " << chassisData.batA << "A" << std::endl;
        logfile << "Battery Alarm: " << chassisData.batAlarm << std::endl;
        logfile << "Battery Temperature 1: " << chassisData.batTemp1 << std::endl;
        logfile << "Battery Temperature 2: " << chassisData.batTemp2 << std::endl;
        logfile << "Car Speed: " << chassisData.carSpeed << std::endl;
        logfile << "Encoder Value: " << chassisData.encoderValue << std::endl;
        logfile << "Body Temperature: " << static_cast<int>(chassisData.bodyTemperature) << std::endl;
        logfile << "Body Humidity: " << static_cast<int>(chassisData.bodyHumidity) << std::endl;
        logfile << "Angle X: " << chassisData.angleX << std::endl;
        logfile << "Angle Y: " << chassisData.angleY << std::endl;
        logfile << "Angle Z: " << chassisData.angleZ << std::endl;

        // 写入日志分隔符
        logfile << "------------------------" << std::endl;
        std::cout<<"end!!!!"<<std::endl;
        if(!logfile){
            std::cerr<<"failed to write log!!!"<<std::endl;
        }

    }
    


    void parseChassisData() {
        std::cout<<"parseChassisData!!!!"<<std::endl;
        // 解析电池容量百分比
        chassisData.batSOC = (receivedData[6] << 8) | receivedData[7];

        // 解析电池当前电压
        chassisData.batV = (receivedData[8] << 8) | receivedData[9];

        // 解析电池当前电流
        chassisData.batA = (receivedData[10] << 8) | receivedData[11];

        // 解析电池故障码
        chassisData.batAlarm = (receivedData[12] << 8) | receivedData[13];

        // 解析电池组温度1
        chassisData.batTemp1 = (receivedData[14] << 8) | receivedData[15];

        // 解析电池组温度2
        chassisData.batTemp2 = (receivedData[16] << 8) | receivedData[17];

        // 解析车速
        chassisData.carSpeed = (receivedData[18] << 8) | receivedData[19];

        // 解析编码器值
        chassisData.encoderValue = (receivedData[20] << 24) | (receivedData[21] << 16) | (receivedData[22] << 8) | receivedData[23];

        // 解析本体温度
        chassisData.bodyTemperature = receivedData[24];

        // 解析本体湿度
        chassisData.bodyHumidity = receivedData[25];

        // 解析倾角X
        chassisData.angleX = (float)(((int32_t)(receivedData[26] << 24) | (receivedData[27] << 16) | (receivedData[28] << 8) | receivedData[29])) / 100.0;

        // 解析倾角Y
        chassisData.angleY = (float)(((int32_t)(receivedData[30] << 24) | (receivedData[31] << 16) | (receivedData[32] << 8) | receivedData[33])) / 100.0;

        // 解析倾角Z
        chassisData.angleZ = (float)(((int32_t)(receivedData[34] << 24) | (receivedData[35] << 16) | (receivedData[36] << 8) | receivedData[37])) / 100.0;

        // 将解析后的数据存储到日志或执行其他操作
        // ...
    }

public:
    ChassisData chassisData;
    std::ofstream logfile;
};

int main() {
    int sockfd;
    struct sockaddr_in serverAddr, clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);
    ChassisParser chassisParser ;

    // 创建UDP套接字
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        std::cerr << "Failed to create socket!" << std::endl;
        return -1;
    }

    // 设置服务器地址信息
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SERVER_PORT);
    if (inet_pton(AF_INET, SERVER_IP, &(serverAddr.sin_addr)) <= 0) {
        std::cerr << "Failed to set server address!" << std::endl;
        return -1;
    }

    // 绑定套接字到本地地址和端口
    if (bind(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == -1) {
        std::cerr << "Failed to bind socket!" << std::endl;
        return -1;
    }

    std::cout << "UDP server is listening on " << SERVER_IP << ":" << SERVER_PORT << std::endl;



    // 接收数据
    ssize_t numBytesReceived;
    while (true) {
        numBytesReceived = recvfrom(sockfd, receivedData, sizeof(receivedData), 0,
                                    (struct sockaddr *)&clientAddr, &clientAddrLen);

        if (numBytesReceived == -1) {
            std::cerr << "Failed to receive data!" << std::endl;
            break;
        } else if (numBytesReceived == 0) {
            std::cout << "Connection closed by the client." << std::endl;
            break;
        } else {
            std::cout << "Received " << numBytesReceived << " bytes of data from "
                      << inet_ntoa(clientAddr.sin_addr) << ":" << ntohs(clientAddr.sin_port) << std::endl;

            // 在这里处理接收到的数据
            // 可以根据需要进行解析、验证或执行其他操作
            // 例如，打印接收到的数据内容
            std::cout << "Received data: ";
            for (size_t i = 0; i < numBytesReceived; ++i) {
                std::cout << std::hex << static_cast<int>(receivedData[i]) << " ";
            }
            chassisParser.parseChassisData();
            chassisParser.logChassisData();
            // chassisParser.closeLog();

                    
            std::cout << std::endl;
        }
    }
    
    // 关闭套接字
    close(sockfd);

    return 0;
}



void parseAndSaveChassisData() {
    // 获取当前时间
    time_t currentTime;
    time(&currentTime);
    struct tm* timeInfo = localtime(&currentTime);

    // 打开日志文件
    FILE* logFile = fopen("log.txt", "a");
    if (logFile == NULL) {
        printf("无法打开日志文件。\n");
        return;
    }

    // 解析并保存数据
    unsigned char receivedData[50]; // 假设接收到的数据存储在receivedData数组中

    // 保存时间戳
    fprintf(logFile, "[%02d:%02d:%02d] Chassis Data:\n", timeInfo->tm_hour, timeInfo->tm_min, timeInfo->tm_sec);

    // 解析和保存数据
    for (unsigned char i = 0; i < 50; i++) {
        fprintf(logFile, "%02X ", receivedData[i]);
    }
    fprintf(logFile, "\n");

    // 关闭日志文件
    fclose(logFile);
}
