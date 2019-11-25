// inlcude ROS library
#include <ros/ros.h>
#include <ros/console.h>
#include <string>
// inlcude LCM library
#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm.h>
// Include ROS message_type which will be published
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
// include LCM message --> user definition
#include "repub_ros/lcm/eurecar/tram_mission.hpp"


class LCMToROS_TRAM_MISSION
{
    public:
        LCMToROS_TRAM_MISSION(ros::NodeHandle& n);
        ~LCMToROS_TRAM_MISSION();
        void lcmCallback(const lcm::ReceiveBuffer* rbuf,
                const std::string& channel_name,
                const eurecar::tram_mission* msg);
        
    private:        
        ros::NodeHandle nh;
        ros::Publisher local_path_pub;
        ros::Publisher tram_mission_pub;

        std_msgs::Float64MultiArray mission_msg;
        nav_msgs::Path path_msg;
        geometry_msgs::PoseStamped pos;
        int data_size;


};

LCMToROS_TRAM_MISSION::LCMToROS_TRAM_MISSION(ros::NodeHandle& n) 
{
    nh = n;
    // Publisher/s
    local_path_pub = nh.advertise<nav_msgs::Path>("TRAM_local_path", 10);
    tram_mission_pub = nh.advertise<std_msgs::Float64MultiArray>("TRAM_MISSION", 10);

    ROS_DEBUG("TRAM_MISSION publisher created");
};

LCMToROS_TRAM_MISSION::~LCMToROS_TRAM_MISSION() 
{    
    ROS_INFO("TRAM_MISSION LCMToROS_TRAM_MISSION destructor.");
}

void LCMToROS_TRAM_MISSION::lcmCallback(const lcm::ReceiveBuffer* rbuf,
        const std::string& channel_name,
        const eurecar::tram_mission* msg)
{
    // // ROS_DEBUG("Received message on channel \"%s\"", channel_name.c_str());
    data_size = 10;
    double contents[data_size];

    for(int n =0; n< data_size; n++){
        contents[n] = 0.0;
    }

    mission_msg.data.clear();
    path_msg.poses.clear();

    contents[0] = (double)msg -> current_idx;
    contents[1] = (double)msg -> mission_number; 
    contents[2] = (double)msg -> num_localpath; //array size of local path
    contents[3] = msg -> pos_x; //unit: m
    contents[4] = msg -> pos_y; //Unit: m
    contents[5] = msg -> cur_veh_heading; //Unit: rad
    contents[6] = msg -> cur_veh_speed; // Unit: km/h
    contents[7] = msg -> cur_veh_steer_angle;
    contents[8] = msg -> goal_veh_speed;
    contents[9] = msg -> goal_veh_steer_angle;
    for(int j=0; j< data_size; j++){
        mission_msg.data.push_back(contents[j]);
    }
    tram_mission_pub.publish(mission_msg);

    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "/base_footprint";
    std::vector<geometry_msgs::PoseStamped> poses((int)contents[2]);

    for(int i=1; i< (int)contents[2]; i++){
        poses.at(i).header.stamp = ros::Time::now();
        poses.at(i).header.frame_id = "/base_footprint";
        poses.at(i).pose.position.x = msg->pt_x[i];
        poses.at(i).pose.position.y = msg->pt_y[i];
    }
    path_msg.poses = poses;
    // std::cout << path_msg << std::endl;
    local_path_pub.publish(path_msg);
};
        

int main(int argc, char** argv)
{
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;

    ros::init(argc, argv, "TRAM_MISSION_republish");
    ros::NodeHandle nh;   

    LCMToROS_TRAM_MISSION handlerObject(nh);
    lcm.subscribe("TRAM_MISSION", &LCMToROS_TRAM_MISSION::lcmCallback, &handlerObject);

    int lcm_timeout = 100; //ms
    while( ( lcm.handleTimeout(lcm_timeout) >= 0 ) && (ros::ok()) ) //
        ros::spinOnce();

    return 0;
}
