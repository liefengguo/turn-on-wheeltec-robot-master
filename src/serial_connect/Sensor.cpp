#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Int32.h>
// 串口参数
#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE 9600

// 控制指令
#define CMD_LENGTH 5
const uint8_t CMD[CMD_LENGTH] = {0x55, 0xAA, 0x01, 0x01, 0x00};

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

int main(int argc, char **argv) {
    ros::init(argc, argv, "rs485_node");
    ros::NodeHandle nh;

    // 初始化串口
    serial::Serial ser;
    ser.setPort(SERIAL_PORT);
    ser.setBaudrate(BAUDRATE);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(timeout);
    ser.open();
    if (!ser.isOpen()) {
        ROS_ERROR("Failed to open serial port.");
        return -1;
    }

    while (ros::ok()) {
        // 发送控制指令
        ser.write(CMD, CMD_LENGTH);

        // 读取返回数据
        uint8_t buffer[FRAME_LENGTH];
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
        ros::Publisher pub = nh.advertise<std_msgs::Int32>("probe_1", 1000);
        std_msgs::Int32 msg;
        uint16_t data1 = (frame->data[0] << 8) | frame->data[1];
        int value = (int)data1;
        msg.data = value;
        pub.publish(msg);
    }

    ser.close();
    return 0;
}
