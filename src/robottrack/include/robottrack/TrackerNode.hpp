#ifndef _TRACKER_NODE_HPP
#define _TRACKER_NODE_HPP

#include <iostream>
#include "Eigen/Core"
#include "ros/ros.h"
#include "robotinterfaces/Armor.h"
#include "robotinterfaces/Armors.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tracker.hpp"
#include "std_srvs/Trigger.h"


class TrackerNode
{
private:
    geometry_msgs::TransformStamped transformStamped;
    ros::Subscriber armors_sub;
    ros::Publisher target_pub;
    ros::ServiceServer reset_tracker_srv;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    std::string target_frame;
    double max_armor_distance;
    double max_match_distance;
    double max_match_yaw_diff;

    std::unique_ptr<Tracker> tracker;
    ros::Time last_time;
    double dt;

    double lost_time_thres;
    double s2qxyz, s2qyaw, s2qr;
    double r_xyz_factor, r_yaw;
public:
    TrackerNode(/* args */) = delete;
    TrackerNode(ros::NodeHandle nh);
    ~TrackerNode();

    void armorsCallback(const robotinterfaces::Armors::Ptr armors_msg);
    // 不知道有什么用
    bool resetTrackerCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

};

#endif // !_TRACKER_NODE_HPP