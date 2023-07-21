#include "../../include/radar_controller/RadarController.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "radarController");
    VRFKReaderPtr vrtkReader = boost::make_shared<VRFKReader>();
    DistanceSensor sensor;
    RadarController radar_controller;
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
            radar_controller.setGNSSStatus(vrtkReader->getGNSSStatus());
            radar_controller.controlByRadar();
        }
        ros::spinOnce();
        loop_rate.sleep();//以20Hz循环，循环跑太快就在这里睡一会儿
    }
    // ros::spin();
    return 0;
}
