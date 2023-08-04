#include "../../include/radar_controller/a05_radar.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_monitor");
    ros::NodeHandle nh;

    RadarController_a05 radarController_a05;

    ros::spin();

    return 0;
}