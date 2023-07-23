#include "../../include/serial_connect/six_decition_radar.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_monitor");
    ros::NodeHandle nh;

    DistanceSensor sensor;
    ros::spin();

    return 0;
}