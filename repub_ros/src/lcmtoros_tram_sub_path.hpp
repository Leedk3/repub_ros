// inlcude ROS library
#include <ros/ros.h>
#include <ros/console.h>
#include <string>
// inlcude LCM library
#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm.h>
// Include ROS message_type which will be published
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
// include LCM message --> user definition
#include "repub_ros/lcm/eurecar/tram_sub_path.hpp"


class LCMToROS_TRAM_SUB_PATH
{
    public:
        LCMToROS_TRAM_SUB_PATH(ros::NodeHandle& n);
        ~LCMToROS_TRAM_SUB_PATH();
        void lcmCallback(const lcm::ReceiveBuffer* rbuf,
                const std::string& channel_name,
                const eurecar::tram_sub_path* msg);
        
    private:        
        ros::NodeHandle nh;
        ros::Publisher sub_path_pub;
        ros::Publisher sub_maptrack2_pub; 
        //tram sub path publisher, Don't confuse between sub and pub

        geometry_msgs::Point sub_maptrack2_point;
        geometry_msgs::Point sub_path_point;
        
        visualization_msgs::Marker sub_maptrack2_marker;
        visualization_msgs::Marker sub_path_marker;
        
};

LCMToROS_TRAM_SUB_PATH::LCMToROS_TRAM_SUB_PATH(ros::NodeHandle& n) 
{
    nh = n;
    // Publisher/s
    sub_path_pub = nh.advertise<visualization_msgs::Marker>("TRAM_sub_path", 10);
    sub_maptrack2_pub = nh.advertise<visualization_msgs::Marker>("TRAM_sub_maptrack2", 10);

    ROS_DEBUG("tram_sub_path publisher created");
};

LCMToROS_TRAM_SUB_PATH::~LCMToROS_TRAM_SUB_PATH() 
{    
    ROS_INFO("LCMToROS_TRAM_SUB_PATH destructor.");
}

void LCMToROS_TRAM_SUB_PATH::lcmCallback(const lcm::ReceiveBuffer* rbuf,
        const std::string& channel_name,
        const eurecar::tram_sub_path* msg)
{
    // // ROS_DEBUG("Received message on channel \"%s\"", channel_name.c_str());
    sub_path_marker.points.clear();
    sub_maptrack2_marker.points.clear();

    //SUB MAPTRACK
    sub_maptrack2_marker.header.stamp = ros::Time::now();
    sub_maptrack2_marker.header.frame_id = "/base_gps";
    // std::vector<geometry_msgs::PoseStamped> poses((int)contents[2]);
    sub_maptrack2_marker.type = visualization_msgs::Marker::LINE_STRIP;
    sub_maptrack2_marker.pose.orientation.x = 0.0;
    sub_maptrack2_marker.pose.orientation.y = 0.0;
    sub_maptrack2_marker.pose.orientation.z = 0.0;
    sub_maptrack2_marker.pose.orientation.w = 1.0;
    sub_maptrack2_marker.scale.x = 0.1; sub_maptrack2_marker.scale.y = 0.1; sub_maptrack2_marker.scale.z = 0.1;
    sub_maptrack2_marker.color.r = 0; sub_maptrack2_marker.color.g = 1; sub_maptrack2_marker.color.b = 1;
    sub_maptrack2_marker.color.a = 1;

    for(int i=2; i< msg->num_sub_maptrack2; i++){
        sub_maptrack2_point.x = msg->sub_maptrack2_x[i];
        sub_maptrack2_point.y =  - msg->sub_maptrack2_y[i];
        sub_maptrack2_point.z = 0;
        sub_maptrack2_marker.points.push_back(sub_maptrack2_point);
    }
    sub_maptrack2_pub.publish(sub_maptrack2_marker);

    //SUB PATH
    sub_path_marker.header.stamp = ros::Time::now();
    sub_path_marker.header.frame_id = "/base_gps";
    // std::vector<geometry_msgs::PoseStamped> poses((int)contents[2]);
    sub_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    sub_path_marker.pose.orientation.x = 0.0;
    sub_path_marker.pose.orientation.y = 0.0;
    sub_path_marker.pose.orientation.z = 0.0;
    sub_path_marker.pose.orientation.w = 1.0;
    sub_path_marker.scale.x = 0.1; sub_path_marker.scale.y = 0.1; sub_path_marker.scale.z = 0.1;
    sub_path_marker.color.r = 1; sub_path_marker.color.g = 1; sub_path_marker.color.b = 0;
    sub_path_marker.color.a = 1;

    for(int i=3; i< msg->num_sub_path; i++){
        sub_path_point.x = msg->sub_path_x[i];
        sub_path_point.y =  msg-> sub_path_y[i];
        sub_path_point.z = 0;
        sub_path_marker.points.push_back(sub_path_point);
    }
    sub_path_pub.publish(sub_path_marker);


};