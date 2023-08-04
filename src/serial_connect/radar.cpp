#include <ros/ros.h>
#include <serial/serial.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <std_msgs/Int32.h>
#include <turn_on_wheeltec_robot/a22_data.h>
using namespace turn_on_wheeltec_robot;

// 串口参数
#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE 9600

// 控制指令
#define CMD_LENGTH 5
const uint8_t CMD[CMD_LENGTH] = {0x55, 0xAA, 0x01, 0x01, 0x01};

// 接收数据帧结构
#define FRAME_LENGTH 13
#define HEADER 0x55AA

typedef struct {
    uint16_t header;  // 帧头
    uint8_t address;  // 地址
    uint8_t command;  // 指令
    uint8_t data[8];  // 数据
    uint8_t checksum; // 校验位
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
            ser.write(CMD, CMD_LENGTH);
        }
        cv.notify_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

// 接收指令线程函数
void receiveThreadFunc(serial::Serial &ser, std::mutex &mutex, std::condition_variable &cv, bool &isRunning, ros::Publisher &pub) {
    uint8_t buffer[FRAME_LENGTH];
    while (isRunning) {
        // 等待发送指令
        {
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock);
        }

        // 读取返回数据
        int count = ser.read(buffer, FRAME_LENGTH);
        // if (count != FRAME_LENGTH) {
        //     ROS_ERROR("Failed to read response.");
        //     continue; // 读取失败，跳过此次循环
        // }

        // 解析数据
        Frame *frame = (Frame *)buffer;
        // if (frame->header != HEADER) {
        //     ROS_ERROR("Invalid response header.");
        //     continue; // 解析失败，跳过此次循环
        // }
        uint8_t checksum = calculateChecksum(buffer + 2, FRAME_LENGTH - 3);
        // if (checksum != frame->checksum) {
        //     ROS_ERROR("Invalid checksum.");
        //     continue; // 校验失败，跳过此次循环
        // }

        // 发布ROS话题
        a22_data a22_data;
        uint16_t data1 = (frame->data[0] << 8) | frame->data[1];
        uint16_t data2 = (frame->data[2] << 8) | frame->data[3];
        uint16_t data3 = (frame->data[4] << 8) | frame->data[5];
        uint16_t data4 = (frame->data[6] << 8) | frame->data[7];
        std::vector<int32_t> datas;
        datas.push_back(data1);
        datas.push_back(data2);
        datas.push_back(data3);
        datas.push_back(data4);
        int value1 = (int)data1;
        int value2 = (int)data2;
        int value3 = (int)data3;
        int value4 = (int)data4;
        std::cout<<"1号："<<data1<<"2:"<<data2<<"3:"<<data3<<"4:"<<data4<<std::endl;
        a22_data.a22_datas = datas;
        pub.publish(a22_data);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rs485_node");
    ros::NodeHandle nh;

    // 创建ROS话题
    ros::Publisher pub = nh.advertise<a22_data>("a05_radar", 1000); //因为a22_data 的格式符合a05需求 所以用了这个类型

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

   
