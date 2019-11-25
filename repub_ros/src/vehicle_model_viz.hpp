// inlcude ROS library
#include <ros/ros.h>
#include <ros/console.h>
#include <string>
// Include ROS message_type which will be published
#include "visualization_msgs/Marker.h"


class VEHICLE_MODEL_VIZ
{
    public:
        VEHICLE_MODEL_VIZ(ros::NodeHandle& n);
        
        ~VEHICLE_MODEL_VIZ();
        
        
    private:        
        ros::NodeHandle nh;
        ros::Publisher vehicle_marker_pub;
        visualization_msgs::Marker vehicle_marker;
        void Vehicle_Publisher();
};

VEHICLE_MODEL_VIZ::VEHICLE_MODEL_VIZ(ros::NodeHandle& n) 
{
    nh = n;
    // Publisher/s
    vehicle_marker_pub = nh.advertise<visualization_msgs::Marker>("VEHICLE_MARKER", 10);
    vehicle_marker_pub.publish(vehicle_marker);
    ROS_DEBUG("vehicle marker publisher created");
};

VEHICLE_MODEL_VIZ::~VEHICLE_MODEL_VIZ() 
{    
    // ROS_INFO("VEHICLE_MODEL_VIZ destructor.");
}

void VEHICLE_MODEL_VIZ::Vehicle_Publisher()
{
    // // ROS_DEBUG("Received message on channel \"%s\"", channel_name.c_str());
    vehicle_marker.header.stamp = ros::Time::now();
    vehicle_marker.header.frame_id = "/base_footprint";
    // std::vector<geometry_msgs::PoseStamped> poses((int)contents[2]);
    vehicle_marker.type = visualization_msgs::Marker::CUBE;
    vehicle_marker.pose.orientation.x = 0.0;
    vehicle_marker.pose.orientation.y = 0.0;
    vehicle_marker.pose.orientation.z = 0.0;
    vehicle_marker.pose.orientation.w = 1.0;
    vehicle_marker.scale.x = 3; vehicle_marker.scale.y = 5; vehicle_marker.scale.z = 2.5;
    vehicle_marker.color.r = 0.5; vehicle_marker.color.g = 0.5; vehicle_marker.color.b = 0;
    vehicle_marker.color.a = 1;
    
};