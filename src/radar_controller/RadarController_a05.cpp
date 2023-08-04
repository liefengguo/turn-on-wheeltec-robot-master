#include "../../include/radar_controller/RadarController.h"

RadarController_a05::RadarController_a05() {
    sub_ = nh_.subscribe("a05_radar", 1, &RadarController_a05::distanceCallback, this);
    radar_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 20);
    nh.param<int>("targetDistance", targetDistance, 230);
    nh.param<int>("flag", flag, 1);
    nh.param<bool>("log_flag", log_flag, 1);
    nh.param<int>("distanceMax_radar2", distanceMax_radar2, 399);
    nh.param<int>("distanceMax_radar1", distanceMax_radar1, 300);
    nh.param<int>("distanceMax_radar3", distanceMax_radar3, 399);

    if(log_flag){
        std::string path = "/home/glf/log/";
        std::stringstream  filename;
        auto now = std::chrono::system_clock::now();
        std::time_t timestamp = std::chrono::system_clock::to_time_t(now);
        struct tm* timeinfo = std::localtime(&timestamp);

        filename <<path<< "RadarController_a05" << (timeinfo->tm_mon + 1) 
        << "-" << timeinfo->tm_mday<<"-" << timeinfo->tm_hour <<"-" << timeinfo->tm_min
                     << ".txt";
        // filename <<path<< "RadarController_a05"  << ".txt";
        std::cout<<"file path :"<<filename.str()<<std::endl;
        // 打开日志文件
        logfile.open(filename.str(), std::ios::app);
    }
}
RadarController_a05::~RadarController_a05() {
    vel_msg.angular.z = 0;  
    vel_msg.linear.x = 0;
    radar_cmd_vel.publish(vel_msg);
    radar_cmd_vel.publish(vel_msg);
    radar_cmd_vel.publish(vel_msg);
    if(log_flag){
        if (logfile.is_open()) {
            std::cout<<"close!!!!"<<std::endl;
            logfile.close();
        }
    }
}
void distanceCallback(const std_msgs::Int32::ConstPtr& msg) {
    distance_ = msg->data;
    static AdaptiveFilter filter(bufferSize,threshold);  

    int filteredDistance = filter.filter(distance_);  // 应用自适应滤波器
    std::cout<<"真值："<<distance_<< "距离："<<filteredDistance<<std::endl;

    // 判断距离是否偏离目标距离范围
    if (filteredDistance < targetDistance - distanceThreshold) {
        // 距离过近，需要向左调整车辆行驶方向
        // 在这里添加调整车辆方向的代码
        std::cout<< "距离过近，left:"<<filteredDistance - targetDistance<<std::endl;

    } else if (filteredDistance > targetDistance + distanceThreshold) {
        // 距离过远，需要向左调整车辆行驶方向
        // 在这里添加调整车辆方向的代码
        std::cout<< "distance too far， right please!!"<<filteredDistance - targetDistance<<std::endl;
    } else {
        // 距离在目标范围内，维持当前行驶方向
        // 在这里添加维持当前行驶方向的代码
        std::cout<< "OK! go "<<std::endl;
    }
}
void RadarController_a05::setLog_cur_time(){
    auto now = std::chrono::system_clock::now();
    std::time_t cur_timestamp = std::chrono::system_clock::to_time_t(now);
    logfile << cur_timestamp << " ";
}
void RadarController_a05::getStop_flag(){
    auto now = std::chrono::system_clock::now();
    std::time_t cur_timestamp = std::chrono::system_clock::to_time_t(now);
    logfile << cur_timestamp << " ";
}
void RadarController_a05::setRadar1(int value) {
    radar1 = value;
    if(log_flag){
        setLog_cur_time();
        logfile << value << " ";
    }

}

void RadarController_a05::setRadar2(int value) {
    radar2 = value;
    if(log_flag){
        logfile << value << " ";
    }
}

void RadarController_a05::setRadar3(int value) {
    radar3 = value;
    if(log_flag){
        logfile << value << " ";
    }
}

void RadarController_a05::setRadar4(int value) {
    radar4 = value;
    if(log_flag){
        logfile << value << " ";
    }
}

void RadarController_a05::setRadar5(int value) {
    radar5 = value;
    if(log_flag){
        logfile << value << " ";
    }
}

void RadarController_a05::setRadar6(int value) {
    radar6 = value;
    if(log_flag){
        logfile << value << " ";
    }
}
bool RadarController_a05::getStop_flag() {
    return stop_flag;
}
void RadarController_a05::stop_cmd() {
    vel_msg.angular.z = 0;  // 设置正角速度以右转
    vel_msg.linear.x = 0;
    radar_cmd_vel.publish(vel_msg);
    if(log_flag){
        logfile << vel_msg.angular.z << std::endl;
    }
}
void RadarController_a05::controlByRadar() {
    if ( flag) {
        if (radar1 <  distanceMax_radar1 || radar2 < distanceMax_radar2 || radar3 < distanceMax_radar3 || radar4 < distanceMax_radar4 || radar5 < distanceMax_radar5 || radar6  < distanceMax_radar6 ){
            stop_cmd();
            stop_flag = 1;
        } else {
            stop_flag  = 0;
        }
        
    } else {
        // RTK is not good, use alternative control method
        // Add your code here for alternative control method
    }
    // if ()
    std::cout<<"path_degree:" << path_degree<<std::endl;
}

