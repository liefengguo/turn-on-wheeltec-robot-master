#include "../include/radar_controller/RadarController.h"
#include "../include/common/purepursuit.hpp"

int main(int argc, char**argv){
    ros::init(argc, argv, "radar_rtk_Controller");
    VRFKReader vrtkReader ;
    DistanceSensor sensor;  
    PurePursuit controller_path;
    RadarController radar_controller;
    // controller_path.run();
    ros::Rate loop_rate(5);
    while (ros::ok())
    {
        if(sensor.sub_.getNumPublishers() != 0 && controller_path.stop_count==0){                    // 判断是否有接收到超声波话题
            radar_controller.setRadar1(sensor.getFilteredDistance0());
            radar_controller.setRadar2(sensor.getFilteredDistance1());
            radar_controller.setRadar3(sensor.getFilteredDistance2());
            radar_controller.setRadar4(sensor.getFilteredDistance3());
            radar_controller.setRadar5(sensor.getFilteredDistance4());
            radar_controller.setRadar6(sensor.getFilteredDistance5());
            radar_controller.setGNSSStatus(vrtkReader.getGNSSStatus());
            radar_controller.setPath_degree(controller_path.getDegree());
            radar_controller.setPath_vel(controller_path.x_bei);
            radar_controller.setPath_dis(0);
            if(!controller_path.in_bizhang)  // 不处于避障状态下
                radar_controller.controlByRadar();
        }
        else if(controller_path.stop_count>0)
        {
            controller_path.stop_count -= 1;
            cout << "啥也不干，停下等着" << endl;
            if(controller_path.stop_count==0)
            {
                cout << "假设重新检测到了障碍物，重新启动" << endl;
                controller_path.x_up = true;
            }
        }
        if(controller_path.x_up== true) // 慢慢加速
        {
            controller_path.x_bei += 1;
            if(controller_path.x_bei==6)
                controller_path.x_up = false;
        }
        if(controller_path.x_down== true) // 慢慢减速
        {
            controller_path.x_bei -= 1;
            if(controller_path.x_bei==0)
            {
                controller_path.x_down = false;
                if(controller_path.has_barrier == true )
                {
                    controller_path.has_barrier= false;
                    controller_path.dwa_get_trace();
                    controller_path.stop_count = 13;
                }
                if(controller_path.will_stop== true) // 如果此时该停了，就跳出循环
                {
                    break;
                }
            }
        }
        cout << endl;
        ros::spinOnce();
        loop_rate.sleep();//以20Hz循环，循环跑太快就在这里睡一会儿
    }
    // ros::spin();
    return 0;
}
