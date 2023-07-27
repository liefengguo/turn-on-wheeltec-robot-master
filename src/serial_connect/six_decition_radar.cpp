#include "../../include/serial_connect/six_decition_radar.h"

DistanceSensor::DistanceSensor() {
    sub_ = nh_.subscribe("a22_radar", 1, &DistanceSensor::distanceCallback, this);
    nh_.param<int>("targetDistance", targetDistance, 230);
    nh_.param<int>("distanceThreshold", distanceThreshold, 80);
    nh_.param<int>("bufferSize", bufferSize, 20);
    nh_.param<int>("threshold", threshold, 5000);
    nh_.param<int>("log_flag", log_flag, 1);
    if(log_flag){
        std::string path = "/home/glf/log/";
        std::stringstream  filename;
        auto now = std::chrono::system_clock::now();
        std::time_t timestamp = std::chrono::system_clock::to_time_t(now);
        struct tm* timeinfo = std::localtime(&timestamp);

        filename <<path<< "chaoshengbo_log_" << (timeinfo->tm_mon + 1) 
        << "-" << timeinfo->tm_mday<<"-" << timeinfo->tm_hour <<"-" << timeinfo->tm_min
                     << ".txt";
        std::cout<<"file path :"<<filename.str()<<std::endl;

        // filename <<path<< "chaoshengbo"  << ".txt";
        // std::cout<<"file path :"<<filename.str()<<std::endl;
        // 打开日志文件
        logfile.open(filename.str(), std::ios::app);
    }
}
DistanceSensor::~DistanceSensor() {
    if(log_flag){
        if (logfile.is_open()) {
            std::cout<<"close!!!!"<<std::endl;
            logfile.close();
        }
    }
}
void DistanceSensor::filterBigNum(int val,int lastVal){
    
}
void DistanceSensor::distanceCallback(const a22_data::ConstPtr& msg) {
    distance_ = msg->a22_datas;
    
    static AdaptiveFilter filter1(bufferSize,threshold); 
    static AdaptiveFilter filter2(bufferSize,threshold); 
    static AdaptiveFilter filter3(bufferSize,threshold); 
    static AdaptiveFilter filter4(bufferSize,threshold);
    static AdaptiveFilter filter5(bufferSize,threshold);
    static AdaptiveFilter filter6(bufferSize,threshold);

    int lastDistance1,lastDistance2,lastDistance3,lastDistance4,lastDistance5,lastDistance6;
    filteredDistance0 = filter1.filter(distance_[0]);
    filteredDistance1 = filter2.filter(distance_[1]);
    filteredDistance2 = filter3.filter(distance_[2]);
    filteredDistance3 = filter4.filter(distance_[3]);  
    filteredDistance4 = filter5.filter(distance_[4]);
    filteredDistance5 = filter6.filter(distance_[5]);

    std::cout<<"真值0："<<distance_[0]<< "距离："<<filteredDistance0<<std::endl;
    std::cout<<"真值1："<<distance_[1]<< "距离："<<filteredDistance1<<std::endl;
    std::cout<<"真值2："<<distance_[2]<< "距离："<<filteredDistance2<<std::endl;
    if(log_flag){
        if (!logfile.is_open()) {
            // 处理无法打开日志文件的情况
            std::cout<<"no open!!!!"<<std::endl;
            // return;
    }
        logfile << distance_[0] << " ";
        logfile << distance_[1] << " ";
        logfile << distance_[2] << " ";
        logfile << distance_[3] << " ";
        logfile << distance_[4] << " ";
        logfile << distance_[5] << " ";
        logfile << filteredDistance0 << " ";
        logfile << filteredDistance1 << " ";
        logfile << filteredDistance2 << " ";
        logfile << filteredDistance3 << " ";
        logfile << filteredDistance4 << " ";
        logfile << filteredDistance5 <<  std::endl;
    }

}

int DistanceSensor::getFilteredDistance1() const {
    return filteredDistance1;
}

int DistanceSensor::getFilteredDistance2() const {
    return filteredDistance2;
}

int DistanceSensor::getFilteredDistance3() const {
    return filteredDistance3;
}

int DistanceSensor::getFilteredDistance4() const {
    return filteredDistance4;
}

int DistanceSensor::getFilteredDistance5() const {
    return filteredDistance5;
}

int DistanceSensor::getFilteredDistance0() const {
    return filteredDistance0;
}


