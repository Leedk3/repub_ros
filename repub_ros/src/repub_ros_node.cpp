// inlcude ROS library
#include <ros/ros.h>
#include <ros/console.h>
#include <string>
// inlcude LCM library
#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm.h>
#include "lcmtoros_tram_mission_t.hpp"
#include "lcmtoros_tram_sub_path.hpp"
#include "lcmtoros_dw_lane_t.hpp"
int main(int argc, char** argv)
{
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;

    ros::init(argc, argv, "LCM_to_ROS_republish");
    ros::NodeHandle nh;   

    //LCM handler 
    LCMToROS_TRAM_MISSION TRAM_mission_handlerObject(nh);
    LCMToROS_TRAM_SUB_PATH TRAM_subpath_handlerObject(nh);
    LCMToROS_LANE_DETECTION LANE_handlerObject(nh);
    
    //lcm subscriber
    lcm.subscribe("TRAM_MISSION", &LCMToROS_TRAM_MISSION::lcmCallback, &TRAM_mission_handlerObject);
    lcm.subscribe("TRAM_SUB_PATH", &LCMToROS_TRAM_SUB_PATH::lcmCallback, &TRAM_subpath_handlerObject);
    lcm.subscribe("DW_LANE_KF", &LCMToROS_LANE_DETECTION::lcmCallback, &LANE_handlerObject);

    int lcm_timeout = 100; //ms
    while( ( lcm.handleTimeout(lcm_timeout) >= 0 ) && (ros::ok()) ) {
        
    } //
            
        ros::spinOnce();

    return 0;
}
