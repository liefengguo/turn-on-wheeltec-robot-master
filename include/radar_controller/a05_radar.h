#ifndef RADARCONTROLLERA05_H
#define RADARCONTROLLERA05_H
#include <geometry_msgs/Twist.h>
#include <turn_on_wheeltec_robot/a22_data.h>

class RadarController_a05 {
private:
    ros::Publisher radar_cmd_vel;
    ros::NodeHandle nh;
    ros::Subscriber sub_;
    int radar1; //右上
    int radar2; //右中
    int radar3; //右下
    int radar4; //左下
    int radar5; //左中
    int radar6; //左上
    int radar7; //左中
    int radar8; //左上
    double angular_z;
    double linear_x;
    geometry_msgs::Twist vel_msg;
    int distanceThreshold;
    double path_dis_threshold;
    int distanceMax_radar3;
    int distanceMax_radar2;
    int distanceMax_radar1;
    int distanceMax_radar4;
    int distanceMax_radar5;
    int distanceMax_radar6;    
    int distanceMax_radar7;
    int distanceMax_radar8;
    int targetDistance;
    void setLog_cur_time();
    void stop_cmd();
    int a05_flag;
    bool log_flag;
    bool stop_flag;
    std::ofstream logfile;
public:
    RadarController();
    ~RadarController();
    void distanceCallback(const a22_data::ConstPtr& msg);
    // RadarController(const DistanceSensor& distanceSensor, const VRFKReader& VRFK_Reader);
    void controlByRadar();
    void setGNSSStatus(int status);
    void setPath_degree(double degree_);
    void setPath_vel(double vel_);
    void setA05_flag(bool flag_);
    void setPath_dis(double dis_);
    bool getStop_flag();
};

#endif  // RADARCONTROLLERA05_H
