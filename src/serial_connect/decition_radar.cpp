#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include "../../include/serial_connect/adaptive_filter.h"
#include <std_msgs/Int32.h>

class DistanceSensor {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    int distance_;

    int targetDistance ;  // 目标距离
    int distanceThreshold ;  // 距离阈值
    int bufferSize; 
    int threshold;


public:
    DistanceSensor() {
        sub_ = nh_.subscribe("temperature", 1, &DistanceSensor::distanceCallback, this);


        nh_.param<int>("distance_monitor/targetDistance", targetDistance, 230);
        nh_.param<int>("distance_monitor/distanceThreshold", distanceThreshold, 80);
        nh_.param<int>("distance_monitor/bufferSize", bufferSize, 20);
        nh_.param<int>("distance_monitor/threshold", threshold, 50);
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

    int getDistance() {
        return distance_;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_monitor");
    ros::NodeHandle nh;

    DistanceSensor sensor;
    ros::spin();

    return 0;
}
