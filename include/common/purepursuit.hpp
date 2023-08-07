#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H
#include <ros/ros.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include<tf2_ros/transform_listener.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/PoseStamped.h"
#include "../tools/gnss_coordinate_convert.h"
#include "visualization_msgs/Marker.h"
#include <tf2/convert.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
using namespace std;
#define Ld 1.23  //轴距

class PurePursuit {
public:
    PurePursuit();
    ~PurePursuit();
    void run();
    double getDegree();
    void load_aims();
    void dwa_get_trace(); // 生成新的轨迹
    void save_new_pos();

    bool will_stop;
    int x_bei;
    bool x_up; // 表示逐渐加速
    bool x_down; // 表示逐渐减速
    bool in_bizhang;
    bool has_barrier;
    int stop_count;
    double now_pos_x;
    double now_pos_y;
    int aim_index; // 表示避障找到的目标点的编号
    double end_x,end_y; // 目标点的x,y
    string load_aim_path = "/root/catkin_ws/pursite/info.txt";
    string new_save_path = "/root/catkin_ws/pursite/new_save_path.txt";

private:
    void poseCallback(const nav_msgs::Odometry &currentWaypoint);
    void pointCallback(const nav_msgs::Path &msg);
    bool detect_barrier();
    std::array<float, 3> calQuaternionToEuler(const float x, const float y,
                                              const float z, const float w);
    ros::NodeHandle nh_,nh_private_;
    ros::Subscriber splinePath_sub ;
    ros::Subscriber carPose_sub;
    ros::Publisher pub_vel_;
    ros::Publisher path_pub_;
    nav_msgs::Path path;
    ros::Publisher marker_pub;
    ros::Publisher marker_car_pub;
    ros::Publisher marker_pub_street;
    ros::Publisher marker_pub_fuzhu;
    ros::Publisher marker_pub_ext_car;
    visualization_msgs::Marker marker;
    visualization_msgs::Marker marker_car;
    visualization_msgs::Marker marker_streer;
    visualization_msgs::Marker marker_fuzhu;
    visualization_msgs::Marker marker_ext_car;
    std::ofstream logfile;

    vector<double> r_x_;
    vector<double> r_y_;
    vector<double> r_z_;
    vector<double> r_yam_;

    int pointNum ;  //保存路径点的个数
    int index;
    double car_vel,v_max_,w_max_, delta_max_;
    double lookahead_distance_, position_tolerance_;
    bool log_flag;
    double preview_dis;
    double k ;
    double degree;
    bool goal_reached_, path_loaded_;
    int tmp_count;

    // Other class members and functions as before
    // ...
    std::string pure_pursuit_log_path;
    std::stringstream filename;

};
#endif // PURE_PURSUIT_H
