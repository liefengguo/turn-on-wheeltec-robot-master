#include "../../include/radar_controller/RadarController.h"

RadarController::RadarController() {
    radar_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 20);
    nh.param<int>("targetDistance", targetDistance, 230);
    nh.param<int>("distanceThreshold", distanceThreshold, 80);
    nh.param<double>("angular_z", angular_z, 0.08);
    nh.param<double>("linear_x", linear_x, 0.3);
    nh.param<int>("flag", flag, 1);
    nh.param<bool>("log_flag", log_flag, 1);
    nh.param<int>("distanceMax_radar2", distanceMax_radar2, 300);
    if(log_flag){
        std::string path = "/home/glf/log/";
        std::stringstream  filename;
        filename <<path<< "radarController"  << ".txt";
        std::cout<<"file path :"<<filename.str()<<std::endl;
        // 打开日志文件
        logfile.open(filename.str(), std::ios::app);
    }
}
RadarController::~RadarController() {
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
void RadarController::setGNSSStatus(int status) {
    gnss_status = status;
}
void RadarController::setRadar1(int value) {
    radar1 = value;
    if(log_flag){
        logfile << value << " ";
    }

}

void RadarController::setRadar2(int value) {
    radar2 = value;
    if(log_flag){
        logfile << value << " ";
    }
}

void RadarController::setRadar3(int value) {
    radar3 = value;
    if(log_flag){
        logfile << value << " ";
    }
}

void RadarController::setRadar4(int value) {
    radar4 = value;
    if(log_flag){
        logfile << value << " ";
    }
}

void RadarController::setRadar5(int value) {
    radar5 = value;
    if(log_flag){
        logfile << value << " ";
    }
}

void RadarController::setRadar6(int value) {
    radar6 = value;
    if(log_flag){
        logfile << value << " ";
    }
}

void RadarController::turnLeft() {
    vel_msg.angular.z = -angular_z;  // 设置负角速度以左转
    vel_msg.linear.x = path_vel;
    radar_cmd_vel.publish(vel_msg);
    if(log_flag){
        logfile << vel_msg.angular.z << std::endl;
    }
}
void RadarController::turnRight() {
    vel_msg.angular.z = angular_z;  // 设置正角速度以右转
    vel_msg.linear.x = path_vel;
    radar_cmd_vel.publish(vel_msg);
    if(log_flag){
        logfile << vel_msg.angular.z << std::endl;
    }
}
void RadarController::turnLeft_curva() {
    // if (path_degree >0){
        vel_msg.angular.z = path_degree* 0.5 / 38.6 - 0.1 ;  // 设置负角速度以左转
    // } else{
        // vel_msg.angular.z = path_degree* 0.5 / 38.6 - 0.1 ;  // 设置负角速度以左转
    // }
    vel_msg.linear.x = path_vel;
    radar_cmd_vel.publish(vel_msg);
    if(log_flag){
        logfile << vel_msg.angular.z << std::endl;
    }
}
void RadarController::turnRight_curva() {
    vel_msg.angular.z = path_degree* 0.5 / 38.6 + 0.1;  // 设置正角速度以右转
    vel_msg.linear.x = path_vel;
    radar_cmd_vel.publish(vel_msg);
    if(log_flag){
        logfile << vel_msg.angular.z << std::endl;
    }
}
void RadarController::line_controlByRadar(){
    if (radar2 < targetDistance - distanceThreshold ) {
        std::cout<< "距离过近，left:"<<std::endl;
        turnLeft();
        // std::cout<<"radar2: "<<radar2<<" targetDistance - distanceThreshold: "<<targetDistance - distanceThreshold<<std::endl;
    } else if (radar2 > targetDistance + distanceThreshold) {
        turnRight();
        std::cout<< "distance too far， right please!!"<<std::endl;
        // std::cout<<"radar2: "<<radar2<<" targetDistance + distanceThreshold: "<<targetDistance + distanceThreshold<<std::endl;
    } else {
        vel_msg.angular.z = 0;
        vel_msg.linear.x = path_vel;
        radar_cmd_vel.publish(vel_msg);
        std::cout<< "OK! go "<<std::endl;
        // std::cout<<"radar2: "<<radar2<<" targetDistance - distanceThreshold: "<<targetDistance - distanceThreshold<<std::endl;
        if(log_flag){
            logfile << vel_msg.angular.z << std::endl;
        }
    }
}
void RadarController::curvature_controlByRadar(){
    if (radar2 < targetDistance - distanceThreshold ) {
        std::cout<< "距离过近，left:"<<std::endl;
        turnLeft_curva();
        // std::cout<<"radar2: "<<radar2<<" targetDistance - distanceThreshold: "<<targetDistance - distanceThreshold<<std::endl;
    } else if (radar2 > targetDistance + distanceThreshold) {
        turnRight_curva();
        std::cout<< "distance too far， right please!!"<<std::endl;
        // std::cout<<"radar2: "<<radar2<<" targetDistance + distanceThreshold: "<<targetDistance + distanceThreshold<<std::endl;
    } else {
        vel_msg.angular.z = path_degree* 0.5 / 38.6;
        vel_msg.linear.x = linear_x;
        radar_cmd_vel.publish(vel_msg);
        std::cout<< "OK! go "<<std::endl;
        // std::cout<<"radar2: "<<radar2<<" targetDistance - distanceThreshold: "<<targetDistance - distanceThreshold<<std::endl;
        if(log_flag){
            logfile << vel_msg.angular.z << std::endl;
        }
    }
}
void RadarController::controlByRadar() {
    if (gnss_status < 7 || flag) {
        if( abs(path_degree) < 12 || path_degree == 0){
            if (radar2 < distanceMax_radar2){
                line_controlByRadar();
            }
        }else if(abs(path_degree) >= 12){
            curvature_controlByRadar();
        }
    } else {
        // RTK is not good, use alternative control method
        // Add your code here for alternative control method
    }
    // if ()
    std::cout<<"path_degree:" << path_degree<<std::endl;


}

void RadarController::setPath_degree(double degree_){
    path_degree = degree_;
}
void RadarController::setPath_vel(double vel_){
    path_vel = vel_;
}