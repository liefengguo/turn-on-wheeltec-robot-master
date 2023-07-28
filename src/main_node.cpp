#include "../include/radar_controller/RadarController.h"
#include "../include/common/purepursuit.hpp"

int main(int argc, char**argv){
    ros::init(argc, argv, "radar_rtk_Controller");
    VRFKReader vrtkReader ;
    DistanceSensor sensor;  
    PurePursuit controller_path;
    RadarController radar_controller;
    // controller_path.run();
    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        if(sensor.sub_.getNumPublishers() != 0){                    // 判断是否有接收到超声波话题
            radar_controller.setRadar1(sensor.getFilteredDistance0());
            radar_controller.setRadar2(sensor.getFilteredDistance1());
            radar_controller.setRadar3(sensor.getFilteredDistance2());
            radar_controller.setRadar4(sensor.getFilteredDistance3());
            radar_controller.setRadar5(sensor.getFilteredDistance4());
            radar_controller.setRadar6(sensor.getFilteredDistance5());
            radar_controller.setGNSSStatus(vrtkReader.getGNSSStatus());
            radar_controller.setPath_degree(controller_path.getDegree());
            radar_controller.setPath_vel(controller_path.getVel());
            radar_controller.setPath_dis(controller_path.get_path_dis());
            radar_controller.controlByRadar();
        }

        ros::spinOnce();
        loop_rate.sleep();//以20Hz循环，循环跑太快就在这里睡一会儿
    }
    // ros::spin();
    return 0;
}
