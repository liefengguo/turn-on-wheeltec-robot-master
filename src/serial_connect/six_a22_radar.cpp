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
uint16_t data1 ,data2,data3,data4,data5,data6;
uint16_t last_data1 ,last_data2,last_data3,last_data4,last_data5,last_data6 = 0;


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


// 发送指令线程函数
void sendThreadFunc(serial::Serial &ser, std::mutex &mutex, std::condition_variable &cv, bool &isRunning) {
    while (isRunning) {
        // 发送控制指令
        {
            std::lock_guard<std::mutex> lock(mutex);
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
                usleep(8000); // 5毫秒
            }
        }
        cv.notify_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
//接收数据
void Receive(uint8_t bytedata)
{
	static uint8_t step=0;//状态变量初始化为0 在函数中必须为静态变量
	static uint8_t cnt=0,Buf[FRAME_LENGTH],len,cmd,*data_ptr;
	static uint16_t crc16;
	//进行数据解析 状态机
	switch(step)
	{
	    case 0://接收帧头1状态
	        if(bytedata== 0x01 || bytedata== 0x02|| bytedata== 0x03 || bytedata== 0x04 || bytedata== 0x05 || bytedata== 0x06 )
	        {
	            step++;
	            cnt = 0;
	            Buf[cnt++] = bytedata;
	        }break;
	    case 1://接收帧头2状态
	        if(bytedata== 0x03)
	        {
	            step++;
	            Buf[cnt++] = bytedata;
	        } else
	        {
	            step = 0;
	        }
	        break;
	    case 2://接收数据长度字节状态
            if(bytedata== 0x02){
                step++;
                Buf[cnt++] = bytedata;
                data_ptr = &Buf[cnt];
            }else {
                step = 0;
            }
	        break;
	    case 3://接收命令字节状态
	        Buf[cnt++] = bytedata;
	        if(data_ptr + len == &Buf[cnt])//利用指针地址偏移判断是否接收完len位数据
	        {
	            step++;
	        }
	        break;
	    case 4://接收crc16校验高8位字节
	        step++;
	        crc16 = bytedata;
	        break;
	    case 5://接收crc16校验低8位字节
	        crc16 <<= 8;
	        crc16 += bytedata;
	        if(crc16 == usMBCRC16(Buf,2))//校验正确进入下一状态
	        {
	            step = 0;
	        }
	        else if(bytedata== 0x01 || bytedata== 0x02|| bytedata== 0x03 || bytedata== 0x04 || bytedata== 0x05 || bytedata== 0x06 )
	        {
	            step = 1;
	        }
	        else
	        {
	            step = 0;
	        }
            int16_t head;
            Frame *frame = (Frame *)Buf;
            parseResponse(frame,head);
	        break;
	    default:step=0;break;//多余状态，正常情况下不可能出现
	}
}

void parseResponse(const Frame* frame,int16_t &header_) {
        if (frame->address != 0x03) {
            //  int count = ser.read(buffer, FRAME_LENGTH-1);
             ROS_ERROR("Invalid response header.");
            //  continue; // 解析失败，跳过此次循环
        }
        
        switch (frame->header)
        {
        case HEADER:
            /* code */
            header_ = 1;
            data1 = (frame->data[0] << 8) | frame->data[1];
            last_data1 = data1;
            break;
        case 0x02:
            header_ = 2;
            data2 = (frame->data[0] << 8) | frame->data[1];
            last_data2 = data2;
            break;
        case 0x03:
            /* code */
            header_ = 3;
            data3 = (frame->data[0] << 8) | frame->data[1];
            last_data3 = data3;
            break;
        case 0x04 :
            /* code */
            header_ = 4;
            data4 = (frame->data[0] << 8) | frame->data[1];
            last_data4 = data4;
            break;
        case 0x05:
            /* code */
            header_ = 5;
            data5 = (frame->data[0] << 8) | frame->data[1];
            last_data5 = data5;
            break;
        case 0x06:
            /* code */
            header_ = 6;
            data6 = (frame->data[0] << 8) | frame->data[1];
            last_data6 = data6;
            break;
        default:
            break;
        }
}
// 接收指令线程函数
void receiveThreadFunc(serial::Serial &ser, std::mutex &mutex, std::condition_variable &cv, bool &isRunning, ros::Publisher &pub) {
    while (isRunning) {
        // 等待发送指令
        // {
        //     std::unique_lock<std::mutex> lock(mutex);
        //     cv.wait(lock);
        // }
        data1 = last_data1;
        data2 = last_data2;
        data3 = last_data3;
        data4 = last_data4;
        data5 = last_data5;
        data6 = last_data6;
        uint8_t buffer1[FRAME_LENGTH];
        uint8_t buffer2[FRAME_LENGTH];
        uint8_t buffer3[FRAME_LENGTH];
        uint8_t buffer4[FRAME_LENGTH];
        uint8_t buffer5[FRAME_LENGTH];
        uint8_t buffer6[FRAME_LENGTH];

        // 读取返回数据
        int count1 = ser.read(buffer1, FRAME_LENGTH);
        int count2 = ser.read(buffer2, FRAME_LENGTH);
        int count3 = ser.read(buffer3, FRAME_LENGTH);
        int count4 = ser.read(buffer4, FRAME_LENGTH);
        int count5 = ser.read(buffer5, FRAME_LENGTH);
        int count6 = ser.read(buffer6, FRAME_LENGTH);

        if (count1 != FRAME_LENGTH) {
             ROS_ERROR("Failed to read response.");
            cout<< " count1 :" <<count1<<std::endl;
            //  continue; // 读取失败，跳过此次循环
        }
       for (int i = 0; i < FRAME_LENGTH; ++i) {
           cout<< " buffer1 :"<<static_cast<int>(buffer1[i])<<std::endl;
       }
       for (int i = 0; i < FRAME_LENGTH; ++i) {
           cout<< " buffer2 :" <<static_cast<int>(buffer2[i])<<std::endl;
       }       
       for (int i = 0; i < FRAME_LENGTH; ++i) {
           cout<< " buffer3 :"<<static_cast<int>(buffer3[i])<<std::endl;
       }       
       for (int i = 0; i < FRAME_LENGTH; ++i) {
           cout<< " buffer5 :"<<static_cast<int>(buffer5[i])<<std::endl;
       }       
       for (int i = 0; i < FRAME_LENGTH; ++i) {
           cout<< " buffer4 :" <<static_cast<int>(buffer4[i])<<std::endl;
       }       
       for (int i = 0; i < FRAME_LENGTH; ++i) {
           cout<< " buffer6 :"<<static_cast<int>(buffer6[i])<<std::endl;
       }      

        // 解析数据
        int16_t head1,head2,head3,head4,head5,head6;
        Frame *frame1 = (Frame *)buffer1;
        Frame *frame2 = (Frame *)buffer2;
        Frame *frame3 = (Frame *)buffer3;
        Frame *frame4 = (Frame *)buffer4;
        Frame *frame5 = (Frame *)buffer5;
        Frame *frame6 = (Frame *)buffer6;

        parseResponse(frame1,head1);
        parseResponse(frame2,head2);
        parseResponse(frame3,head3);
        parseResponse(frame4,head4);
        parseResponse(frame5,head5);
        parseResponse(frame6,head6);
        
        // 发布ROS话题
        a22_data a22_data;
        std::vector<int32_t> datas;
        datas.push_back(data1);
        datas.push_back(data2);
        datas.push_back(data3);
        datas.push_back(data4);
        datas.push_back(data5);
        datas.push_back(data6);
        std::cout<<"1号："<<data1<<" 2号："<<data2<<" 3号："<<data3<<" 4号："<<data4<<" 5号："<<data5<<" 6号："<<data6<<std::endl;
        // std::cout<<"1号："<<data1<< " 03:"<< static_cast<int>(frame->data[0]) << " 04:"<<static_cast<int>(frame->data[1])<<std::endl;
        a22_data.a22_datas = datas;
        pub.publish(a22_data);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "a22_radar_node");
    ros::NodeHandle nh;
    uint8_t pucCRCHi , pucCRCLo;
    std::string SERIAL_PORT;
    nh.param<string>("a22_radar_node/SERIAL_PORT", SERIAL_PORT, "/dev/ttyUSB0");
        // nh_.param<int>("distance_monitor/bufferSize", bufferSize, 20);

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

    // cout<<hex<<static_cast<int>(pucCRCHi)<<static_cast<int>(pucCRCLo)<<"result:"<<result<<endl;
    // 初始化串口
    serial::Serial ser(SERIAL_PORT, BAUDRATE, serial::Timeout::simpleTimeout(100));


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