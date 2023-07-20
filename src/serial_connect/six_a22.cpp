#include <ros/ros.h>
#include <serial/serial.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <std_msgs/Int32.h>
#include "../../include/serial_connect/add_CRC.h"
#include <turn_on_wheeltec_robot/a22_data.h>
// 串口参数
// #define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE 115200
using namespace turn_on_wheeltec_robot;

// 控制指令
#define CMD_LENGTH 8
uint8_t CMD_01[CMD_LENGTH] = {0x01, 0x03, 0x01, 0x00, 0x00, 0x01, 0x85,0xF6};
uint8_t CMD_02[CMD_LENGTH] = {0x02, 0x03, 0x01, 0x00, 0x00, 0x01, 0x85,0xF6};
uint8_t CMD_03[CMD_LENGTH] = {0x03, 0x03, 0x01, 0x00, 0x00, 0x01, 0x85,0xF6};
uint8_t CMD_04[CMD_LENGTH] = {0x04, 0x03, 0x01, 0x00, 0x00, 0x01, 0x85,0xF6};
uint8_t CMD_05[CMD_LENGTH] = {0x05, 0x03, 0x01, 0x00, 0x00, 0x01, 0x85,0xF6};
uint8_t CMD_06[CMD_LENGTH] = {0x06, 0x03, 0x01, 0x00, 0x00, 0x01, 0x85,0xF6};

// 接收数据帧结构
#define FRAME_LENGTH 7
#define HEADER 0x01

typedef struct {
    uint8_t header;  // 帧头
    uint8_t address;  // 地址
    uint8_t command;  // 指令
    uint8_t data[2];  // 数据
    uint16_t  checksum; // 校验位
} Frame;


int32_t read_a22(serial::Serial &ser,int i){

    uint8_t buffer[FRAME_LENGTH];
    int32_t a22_val;
    // 读取返回数据
    int count = ser.read(buffer, FRAME_LENGTH);
    if (count != FRAME_LENGTH) {
        ROS_ERROR("Failed to read response.");
        // continue; // 读取失败，跳过此次循环
        a22_val = -1;
    }

    // 解析数据
    Frame *frame = (Frame *)buffer;
    a22_val = (frame->data[0] << 8) | frame->data[1];
    if (frame->address != 0x03) {
        int count = ser.read(buffer, FRAME_LENGTH-1);
        ROS_ERROR("Invalid response header.");
            // continue; // 解析失败，跳过此次循环
        a22_val = -1;
    }
    if(frame->header != i){
        ROS_ERROR("Invalid response header.");
        a22_val = -1;
    }
    
    return a22_val;

}
void get_six_a22(serial::Serial &ser, ros::Publisher &pub){
        
    a22_data a22_data;
    std::vector<int32_t> datas;
    for (int  i = 1; i < 7; i++)
    {
        switch (i)
        {
        case 1:
            ser.write(CMD_01, CMD_LENGTH);
            break;
        case 2:
            ser.write(CMD_02, CMD_LENGTH);
            break;
        case 3:
            ser.write(CMD_03, CMD_LENGTH);
            break;
         case 4:
            ser.write(CMD_04, CMD_LENGTH);
            break;
         case 5:
            ser.write(CMD_05, CMD_LENGTH);
            break;
         case 6:
            ser.write(CMD_06, CMD_LENGTH);
            break;
         default:
            break;
        }
        usleep(30000); // 300毫秒
        datas.push_back(read_a22(ser,i));
    }
        a22_data.a22_datas = datas;
        pub.publish(a22_data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "a22_radar_node");
    ros::NodeHandle nh;
    uint8_t pucCRCHi , pucCRCLo;
    std::string SERIAL_PORT;
    nh.param<string>("a22_radar_node/SERIAL_PORT", SERIAL_PORT, "/dev/ttyUSB0");

    // 创建ROS话题
    ros::Publisher pub = nh.advertise<a22_data>("a22_radar", 1000);
    uint16_t result =  usMBCRC16(CMD_01,CMD_LENGTH - 2,pucCRCHi ,pucCRCLo);
    CMD_01[CMD_LENGTH - 1] = pucCRCHi;
    CMD_01[CMD_LENGTH - 2] = pucCRCLo;
    uint16_t result2 =  usMBCRC16(CMD_02,CMD_LENGTH - 2,pucCRCHi ,pucCRCLo);
    CMD_02[CMD_LENGTH - 1] = pucCRCHi;
    CMD_02[CMD_LENGTH - 2] = pucCRCLo;
    uint16_t result3 =  usMBCRC16(CMD_03,CMD_LENGTH - 2,pucCRCHi ,pucCRCLo);
    CMD_03[CMD_LENGTH - 1] = pucCRCHi;
    CMD_03[CMD_LENGTH - 2] = pucCRCLo;
    uint16_t result4 =  usMBCRC16(CMD_04,CMD_LENGTH - 2,pucCRCHi ,pucCRCLo);
    CMD_04[CMD_LENGTH - 1] = pucCRCHi;
    CMD_04[CMD_LENGTH - 2] = pucCRCLo;
    uint16_t result5 =  usMBCRC16(CMD_05,CMD_LENGTH - 2,pucCRCHi ,pucCRCLo);
    CMD_05[CMD_LENGTH - 1] = pucCRCHi;
    CMD_05[CMD_LENGTH - 2] = pucCRCLo;
    uint16_t result6 =  usMBCRC16(CMD_06,CMD_LENGTH - 2,pucCRCHi ,pucCRCLo);
    CMD_06[CMD_LENGTH - 1] = pucCRCHi;
    CMD_06[CMD_LENGTH - 2] = pucCRCLo;

    // 初始化串口
    serial::Serial ser(SERIAL_PORT, BAUDRATE, serial::Timeout::simpleTimeout(100));
    while (ros::ok)
    {
        get_six_a22(ser, pub);
        ros::spinOnce();
    }
    
    
    // 等待程序结束
    // ros::spin();

    return 0;
}