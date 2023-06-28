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
#include <turn_on_wheeltec_robot/Speed.h>
#include "ros/ros.h"
#define SERVER_IP "127.0.0.1" // 本地IP地址
#define SERVER_PORT 8888      // 监听端口号
uint8_t receivedData[50];

struct ChassisData {
    uint16_t batSOC;
    uint16_t batV;
    uint16_t batA;
    uint16_t batAlarm;
    uint16_t batTemp1;
    uint16_t batTemp2;
    int16_t carSpeed;
    int32_t encoderValue;
    int8_t bodyTemperature;
    int8_t bodyHumidity;
    float angleX;
    float angleY;
    float angleZ;
};

class ChassisParser {
public:
    ChassisParser();
    ~ChassisParser();
    void closeLog();
    void logChassisData();  
    void parseChassisData();

    
public:
    ChassisData chassisData;
    std::ofstream logfile;
};