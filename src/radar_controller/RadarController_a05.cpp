#include "../../include/radar_controller/a05_radar.h"

RadarController_a05::RadarController_a05() {
    sub_ = nh_.subscribe("a05_radar", 1, &RadarController_a05::distanceCallback, this);
    radar_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 20);
    nh.param<int>("targetDistance_a05", targetDistance, 230);
    nh.param<int>("flag_a05", flag, 1);
    nh.param<bool>("log_flag_a05", log_flag, 1);
    nh.param<int>("distanceMax_radar1_a05", distanceMax_radar1, 300);
    nh.param<int>("distanceMax_radar2_a05", distanceMax_radar2, 399);
    nh.param<int>("distanceMax_radar3_a05", distanceMax_radar3, 399);
    nh.param<int>("distanceMax_radar4_a05", distanceMax_radar4, 399);
    nh.param<int>("distanceMax_radar5_a05", distanceMax_radar5, 300);
    nh.param<int>("distanceMax_radar6_a05", distanceMax_radar6, 399);
    nh.param<int>("distanceMax_radar7_a05", distanceMax_radar7, 399);
    nh.param<int>("distanceMax_radar8_a05", distanceMax_radar8, 300);

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
void distanceCallback(const a22_data::ConstPtr& msg) {
    std::vector<int32_t> distance_a05 = msg->a22_datas;
    radar1 = distance_a05[0];
    radar2 = distance_a05[1];
    radar3 = distance_a05[2];
    radar4 = distance_a05[3];
    radar5 = distance_a05[4];
    radar6 = distance_a05[5];
    radar7 = distance_a05[6];
    radar8 = distance_a05[7];

    if (a05_flag) {
        if (radar1 <  distanceMax_radar1 || radar2 < distanceMax_radar2 || radar3 < distanceMax_radar3 || 
            radar4 < distanceMax_radar4 || radar5 < distanceMax_radar5 || radar6  < distanceMax_radar6 || 
            radar7 < distanceMax_radar7 || radar8  < distanceMax_radar8 ){
            stop_cmd();
            stop_flag = 1;
        } else {
            stop_flag  = 0;
        }
    } else {
        // RTK is not good, use alternative control method
        // Add your code here for alternative control method
    }
    if(log_flag){
        setLog_cur_time();
        logfile << radar1 << " " << radar2 << " " << radar3 << " " << radar4 
        << " " << radar5 << " " << radar6 << " " << radar7 << " " << radar8;
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
bool RadarController_a05::getStop_flag() {
    return stop_flag;
}
void RadarController_a05::setA05_flag(bool flag_) {
    a05_flag = flag_;
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
    if (a05_flag) {
        if (radar1 <  distanceMax_radar1 || radar2 < distanceMax_radar2 || radar3 < distanceMax_radar3 || 
            radar4 < distanceMax_radar4 || radar5 < distanceMax_radar5 || radar6  < distanceMax_radar6 || 
            radar7 < distanceMax_radar7 || radar8  < distanceMax_radar8 ){
            stop_cmd();
            stop_flag = 1;
        } else {
            stop_flag  = 0;
        }
    } else {
        // RTK is not good, use alternative control method
        // Add your code here for alternative control method
    }
}

