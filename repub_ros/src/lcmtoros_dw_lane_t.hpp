// inlcude ROS library
#include <ros/ros.h>
#include <ros/console.h>
// inlcude LCM library
#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm.h>
// Include ROS message_type which will be published
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
// include LCM message --> user definition
#include "repub_ros/lcm/eurecar/dw_lane_detection_KF.hpp"

class LCMToROS_LANE_DETECTION
{
    public:
        LCMToROS_LANE_DETECTION(ros::NodeHandle& n);
        ~LCMToROS_LANE_DETECTION();
        void lcmCallback(const lcm::ReceiveBuffer* rbuf,
                const std::string& channel_name,
                const eurecar::dw_lane_detection_KF* msg);
        
    private:        
        ros::NodeHandle nh;
        ros::Publisher left_lane_detection_pub;
        ros::Publisher right_lane_detection_pub;

};

LCMToROS_LANE_DETECTION::LCMToROS_LANE_DETECTION(ros::NodeHandle& n) 
{
    nh = n;
    // Publisher/s
    left_lane_detection_pub = nh.advertise<visualization_msgs::Marker>("LEFT_LANE_DETECTION", 10);
    right_lane_detection_pub = nh.advertise<visualization_msgs::Marker>("RIGHT_LANE_DETECTION", 10);
    ROS_DEBUG("LANE_DETECTION publisher created");
};

LCMToROS_LANE_DETECTION::~LCMToROS_LANE_DETECTION() 
{    
    ROS_INFO("LANE_DETECTION LCMToROS_LANE_DETECTION destructor.");
}

void LCMToROS_LANE_DETECTION::lcmCallback(const lcm::ReceiveBuffer* rbuf,
        const std::string& channel_name,
        const eurecar::dw_lane_detection_KF* msg)
{
    ROS_DEBUG("Received message on channel");
    visualization_msgs::Marker left_marker;
    visualization_msgs::Marker right_marker;

    geometry_msgs::Point left_point_buf;
    geometry_msgs::Point right_point_buf;
    // LEFT LINE DETECTION MARKER
    left_marker.header.frame_id = "base_gps";
    left_marker.header.stamp = ros::Time();
    left_marker.type = visualization_msgs::Marker::LINE_STRIP;
    left_marker.pose.orientation.x = 0.0;
    left_marker.pose.orientation.y = 0.0;
    left_marker.pose.orientation.z = 0.0;
    left_marker.pose.orientation.w = 1.0;
    left_marker.scale.x = 0.1; left_marker.scale.y = 0.1; left_marker.scale.z = 0.1;
    left_marker.color.r = 1; left_marker.color.g = 1; left_marker.color.b = 1;
    left_marker.color.a = 1;
    // RIGHT LINE DETECTION MARKER
    right_marker.header.frame_id = "base_gps";
    right_marker.header.stamp = ros::Time();
    right_marker.type = visualization_msgs::Marker::LINE_STRIP;
    right_marker.pose.orientation.x = 0.0;
    right_marker.pose.orientation.y = 0.0;
    right_marker.pose.orientation.z = 0.0;
    right_marker.pose.orientation.w = 1.0;
    right_marker.scale.x = 0.1; right_marker.scale.y = 0.1; right_marker.scale.z = 0.1;
    right_marker.color.r = 1; right_marker.color.g = 1; right_marker.color.b = 1;
    right_marker.color.a = 1;

    double left_3; double left_2; double left_1; double left_0; double left_min; double left_max; 
    double right_3; double right_2; double right_1; double right_0; double right_min; double right_max; 
    double radius = 0.05; 

    //READ coefficient from LCM communication
    for(int i=0; i< msg-> num_of_lanes; i++)
    {
        // LEFT
        if(msg->lane_pos[i] == -1 && msg->confidence[i] > 2){ 
            left_3 = msg->coef_3[i];
            left_2 = msg->coef_2[i];
            left_1 = msg->coef_1[i];
            left_0 = msg->coef_0[i];
            left_min = msg-> min_y[i];
            left_max = msg-> max_y[i];
            for (int j = left_min; j < 2* left_max; j++)
            {
                double x_buf = j * 0.5;
                double y_buf = left_3 * x_buf * x_buf* x_buf + left_2 * x_buf * x_buf + left_1 * x_buf + left_0;
                
                left_point_buf.x = x_buf;
                left_point_buf.y = -y_buf;
                left_marker.points.push_back(left_point_buf);
            }
        }
        // RIGHT
        else if(msg->lane_pos[i] == 1 && msg->confidence[i] > 2){
            right_3 = msg->coef_3[i];
            right_2 = msg->coef_2[i];
            right_1 = msg->coef_1[i];
            right_0 = msg->coef_0[i];
            right_min = msg-> min_y[i];
            right_max = msg-> max_y[i];
            for (int j = right_min; j < 2* right_max; j++)
            {
                double x_buf = j * 0.5;
                double y_buf = right_3 * x_buf * x_buf* x_buf + right_2 * x_buf * x_buf + right_1 * x_buf + right_0;
                right_point_buf.x = x_buf;
                right_point_buf.y = -y_buf;
                right_marker.points.push_back(right_point_buf);
            }
        }
    }
    left_lane_detection_pub.publish(left_marker);
    right_lane_detection_pub.publish(right_marker);
};