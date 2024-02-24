#ifndef _DETECTNODE_HPP_
#define _DETECTNODE_HPP_

#include "image_transport/publisher.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "robotinterfaces/Armor.h"
#include "robotinterfaces/Armors.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "ArmorDetector.hpp"
#include "PnpSolver.hpp"

class DetectNode
{
private:
    ros::NodeHandle nh;
    ros::Subscriber image_sub;
    robotinterfaces::Armors armors_msg;

    ros::Publisher armors_pub;
    
    std::unique_ptr<ArmorDetector> armor_detector;
    std::unique_ptr<PnPSolver> pnp_solver;
    // debug
    image_transport::ImageTransport debug_armor_it;
    image_transport::Publisher debug_armors_pub;
    sensor_msgs::ImagePtr debug_armor_image_msg;
    
    // pnp
    std::array<double, 9> camera_matrix = {
        1803.238347735879, 0, 720.5298209903385,
        0, 1810.617779964718, 576.5437508806779,
        0, 0, 1};
    std::vector<double> dist_coeffs = {-0.06174336716754254, -0.2032169547702652,
                                       0.002101648037236007, -0.0009242357543944713, 0.9095177156836921};


    // detector
    LightParams l;
    ArmorParams a;
    int binary_thres;
    int detect_color;
    double threshold;
    std::string model_path;
    std::string label_path;
    std::vector<std::string> ignore_classes;
    std::unique_ptr<ArmorDetector> initDetector();
public:
    DetectNode() = delete;
    DetectNode(ros::NodeHandle nh);
    ~DetectNode() { }
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};

#endif