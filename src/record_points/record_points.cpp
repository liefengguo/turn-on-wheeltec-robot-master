#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <string>
#include <sstream>
#include <memory>
#include <std_msgs/String.h>
#include <tf2/convert.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <unistd.h>
#include "../../include/tools/gnss_coordinate_convert.h"
using namespace std;
FILE *fp;
string file_path_ = "/home/glf/catkin_ws/car/";
string file_name_ = "info.txt";

void callbackPosition(const nav_msgs::Odometry& msgs)
{
    double position_x = msgs.pose.pose.position.x;
    double position_y = msgs.pose.pose.position.y;
    double position_z = msgs.pose.pose.position.z;
    double yaw = tf::getYaw(msgs.pose.pose.orientation);
    double x ,y ,z;
    // GaussProjCal(position_x,position_y,&x,&y);
    ecefToEnu(position_x,position_y,position_z,&x,&y,&z);
    
    if(fp == NULL)
    {
	ROS_INFO("open record data file %s failed !!!",(file_path_+file_name_).c_str());
    }
    if(fp != NULL)
    {
        fprintf(fp,"%f %f %f %f \n",x,y,z,yaw);
        // fprintf(fp,"%f %f %f %f \n",x,y,z,yaw);
        fflush(fp);
    }
    ROS_INFO("Position x %f", position_x);
    ROS_INFO("Position y %f", position_y);
    ROS_INFO("Position z %f", position_z);
    ROS_INFO("Position yaw %f", yaw);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "record");
    ros::NodeHandle nh;
    ros::Subscriber position_sub = nh.subscribe("/fixposition/odometry", 1, callbackPosition);
    fp = fopen((file_path_+file_name_).c_str(),"w");
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
   //fflush(fp);    
   //fclose(fp);
   ros::spin();

    return 0;
}

