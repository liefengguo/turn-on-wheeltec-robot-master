#ifndef DECITION_RADAR_H
#define DECITION_RADAR_H

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include "adaptive_filter.h"
#include <std_msgs/Int32.h>
#include <turn_on_wheeltec_robot/a22_data.h>
#include <iostream>
#include <fstream> 
#include <sstream>
#include <boost/shared_ptr.hpp>
using namespace turn_on_wheeltec_robot;

class DistanceSensor {
private:
    ros::NodeHandle nh_;
    std::vector<int32_t> distance_;
    // std::vector<std::vector<int32_t>> a22_buff;
    int targetDistance ;  // 目标距离
    int distanceThreshold ;  // 距离阈值
    int bufferSize; 
    int threshold;
    int log_flag;
    int filteredDistance1 ,filteredDistance2,filteredDistance3,filteredDistance4,filteredDistance5,filteredDistance0;
public:
    ros::Subscriber sub_;
    DistanceSensor();
    ~DistanceSensor();
    void filterBigNum(int val,int lastVal);
    void distanceCallback(const a22_data::ConstPtr& msg);

    int getFilteredDistance1() const;
    int getFilteredDistance2() const;
    int getFilteredDistance3() const;
    int getFilteredDistance4() const;
    int getFilteredDistance5() const;
    int getFilteredDistance0() const;
    std::ofstream logfile;
    // bool flag_data;

};
#endif  // ADAPTIVE_FILTER_H
