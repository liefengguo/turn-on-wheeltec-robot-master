#ifndef RADARCONTROLLER_H
#define RADARCONTROLLER_H
#include "../serial_connect/six_decition_radar.h"
#include <geometry_msgs/Twist.h>
#include "VRFKReader.h"
class RadarController {
private:
    ros::Publisher radar_cmd_vel;
    ros::NodeHandle nh;
    int radar1; //右上
    int radar2; //右中
    int radar3; // 右下
    int radar4; //左下
    int radar5; //左中
    int radar6; //左上
    int gnss_status; // GNSS 状态
    double angular_z;
    double linear_x;
    geometry_msgs::Twist vel_msg;
    int distanceThreshold;
    int distanceMax_radar2;
    int targetDistance;
    double path_degree;
    double path_vel;
    
    void turnLeft();
    void turnRight();
    void turnLeft_curva();
    void turnRight_curva();
    int flag;
    bool log_flag;
    std::ofstream logfile;
    void line_controlByRadar();
    void curvature_controlByRadar();
public:
    RadarController();
    ~RadarController();
    // RadarController(const DistanceSensor& distanceSensor, const VRFKReader& VRFK_Reader);
    void controlByRadar();
    void setGNSSStatus(int status);
    void setRadar1(int value);
    void setRadar2(int value);
    void setRadar3(int value);
    void setRadar4(int value);
    void setRadar5(int value);
    void setRadar6(int value);
    void setPath_degree(double degree_);
    void setPath_vel(double vel_);

};

#endif  // RADARCONTROLLER_H
