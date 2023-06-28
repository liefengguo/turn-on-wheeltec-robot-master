#include "udp_action.h"
#include <ros/ros.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "udp_action");
    ros::NodeHandle n;    
    //     <param name="sub_up" value="0"/> 
    // <param name="sub_out" value="0"/> 
    // <param name="sub_clean" value="0"/> 
    //     <param name="fan" value="0"/> 
    // <param name="pump" value="0"/> 
    // <param name="precursor" value="0"/> 
    // <param name="main_up" value="0"/> 
    // <param name="main_clean" value="0"/> 
    // <param name="big_light" value="0"/> 
    int sub_up,sub_out,sub_clean,fan,pump,precursor,main_up,main_clean,big_light;

    n.param<int>("udp_action/sub_up", sub_up, 0);
    n.param<int>("udp_action/sub_out",sub_out , 0);
    n.param<int>("udp_action/sub_clean", sub_clean, 0);
    n.param<int>("udp_action/fan", fan, 0);
    n.param<int>("udp_action/pump",pump , 0);
    n.param<int>("udp_action/precursor",precursor , 0);
    n.param<int>("udp_action/main_up", main_up, 0);
    n.param<int>("udp_action/main_clean", main_clean, 0);
    n.param<int>("udp_action/big_light", big_light, 0);
    // n.param<int>("udp_action/", , 0);
    // n.param<int>("udp_action/", , 0);
    udp_action robot_action;
    robot_action.main_up(main_up);
    robot_action.main_clean(main_clean);
    robot_action.sub_up(sub_up);
    robot_action.sub_out(sub_out);
    robot_action.sub_clean(sub_clean);
    robot_action.fan(fan);
    robot_action.pump(pump);
    robot_action.big_light(big_light);
    robot_action.precursor(precursor);

    // robot_action.

    // robot_action.cmd_main_clean(2,30);
    return 0;
}
