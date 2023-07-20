#include <ros/ros.h>
#include <serial/serial.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <std_msgs/Int32.h>
#include "../../include/serial_connect/add_CRC.h"

// 串口参数
#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE 115200

// 控制指令
#define CMD_LENGTH 8
uint8_t CMD_01[CMD_LENGTH] = {0x01, 0x03, 0x01, 0x00, 0x00, 0x01, 0x85,0xF6};

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

// 计算校验位
uint8_t calculateChecksum(uint8_t *data, uint8_t length) {
    uint8_t sum = 0;
    for (int i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum & 0xFF;
}

// 发送指令线程函数
void sendThreadFunc(serial::Serial &ser, std::mutex &mutex, std::condition_variable &cv, bool &isRunning) {
    while (isRunning) {
        // 发送控制指令
        {
            std::lock_guard<std::mutex> lock(mutex);
            ser.write(CMD_01, CMD_LENGTH);
        }
        cv.notify_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// 接收指令线程函数
void receiveThreadFunc(serial::Serial &ser, std::mutex &mutex, std::condition_variable &cv, bool &isRunning, ros::Publisher &pub) {
    while (isRunning) {
        // 等待发送指令
        {
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock);
        }
        uint8_t buffer[FRAME_LENGTH];

        // 读取返回数据
        int count = ser.read(buffer, FRAME_LENGTH);
         if (count != FRAME_LENGTH) {
             ROS_ERROR("Failed to read response.");
             continue; // 读取失败，跳过此次循环
         }
//        for (int i = 0; i < FRAME_LENGTH; ++i) {
//            cout<< " buffer :"<<hex <<static_cast<int>(buffer[i])<<std::endl;
//        }

        // 解析数据
        Frame *frame = (Frame *)buffer;
         if (frame->header != HEADER && frame->address != 0x03 && buffer[7] != HEADER) {
             int count = ser.read(buffer, FRAME_LENGTH-1);
             ROS_ERROR("Invalid response header.");
             continue; // 解析失败，跳过此次循环
         }
        // 发布ROS话题
        std_msgs::Int32 msg;
        uint16_t data1 = (frame->data[0] << 8) | frame->data[1];

//        uint16_t data2 = (frame->data[2] << 8) | frame->data[3];
//        uint16_t data3 = (frame->data[4] << 8) | frame->data[5];
//        uint16_t data4 = (frame->data[6] << 8) | frame->data[7];
        int value1 = (int)data1;
//        int value2 = (int)data2;
//        int value3 = (int)data3;
//        int value4 = (int)data4;
        std::cout<<"1号："<<value1<< " 03:"<< static_cast<int>(frame->data[0]) << " 04:"<<static_cast<int>(frame->data[1])<<std::endl;
        msg.data = value1;
        pub.publish(msg);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "a22_radar_node");
    ros::NodeHandle nh;
    uint8_t pucCRCHi , pucCRCLo; 

    // 创建ROS话题
    ros::Publisher pub = nh.advertise<std_msgs::Int32>("temperature", 10);
	uint16_t result =  usMBCRC16(CMD_01,CMD_LENGTH - 2,pucCRCHi ,pucCRCLo);
    CMD_01[CMD_LENGTH - 1] = pucCRCHi;
    CMD_01[CMD_LENGTH - 2] = pucCRCLo;
    // 初始化串口
    serial::Serial ser(SERIAL_PORT, BAUDRATE, serial::Timeout::simpleTimeout(1000));


    // 启动发送和接收线程
    std::mutex mutex;
    std::condition_variable cv;
    bool isRunning = true;
    std::thread sendThread(sendThreadFunc, std::ref(ser), std::ref(mutex), std::ref(cv), std::ref(isRunning));
    std::thread receiveThread(receiveThreadFunc, std::ref(ser), std::ref(mutex), std::ref(cv), std::ref(isRunning), std::ref(pub));

    // 等待程序结束
    ros::spin();

    // 停止发送和接收线程
    isRunning = false;
    cv.notify_all();
    sendThread.join();
    receiveThread.join();

    return 0;
}

   
