#include "DetectNode.hpp"
#include "cv_bridge/cv_bridge.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"robot_detect");
    ros::NodeHandle nh;
    DetectNode detect_node(nh);

    // 开始循环，直到节点关闭
    ros::spin();
    return 0;
}

DetectNode::DetectNode(ros::NodeHandle nh): debug_armor_it(nh)
{

    armors_pub = nh.advertise<robotinterfaces::Armors>("detect_armors", 1);
    // debug
    debug_armors_pub = debug_armor_it.advertise("armors_image", 1);

    image_sub = nh.subscribe("camera_image", 1, &DetectNode::imageCallback, this);

    detect_color = nh.param("detect_color", 0); // 0  for red   1 for blue

    // no need to modify
    threshold = nh.param("threshold", 0.7);
    binary_thres = nh.param("binary_thres", 160);
    
    armor_detector = initDetector();
    pnp_solver = std::make_unique<PnPSolver>(camera_matrix, dist_coeffs);

}

std::unique_ptr<ArmorDetector> DetectNode::initDetector()
{
    l.max_ratio = 0.4;
    l.min_ratio = 0.1;
    l.max_angle = 40;

    a.min_light_ratio = 0.7;
    a.min_small_center_distance = 0.8;
    a.max_small_center_distance = 3.2;
    a.min_large_center_distance = 3.2;
    a.max_large_center_distance = 5.5;
    a.max_angle = 35;
    
    ignore_classes.push_back("negative");
    model_path = ros::package::getPath("robotdetect") + "/model/mlp.onnx";
    label_path = ros::package::getPath("robotdetect") + "/model/label.txt";

    auto detector = std::make_unique<ArmorDetector>(binary_thres, detect_color, l, a);
    detector->classifier = std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

    return detector;
}

void DetectNode::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        // 将ROS图像消息转换成OpenCV图像格式
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

        auto armors = armor_detector->work(image);
        // debug
        armor_detector->drawResults(image);
        debug_armor_image_msg = cv_bridge::CvImage(msg->header, "bgr8", image).toImageMsg();
        debug_armors_pub.publish(debug_armor_image_msg);

        robotinterfaces::Armor armor_msg;
        armors_msg.header = msg->header;
        armors_msg.armors.clear(); // 这个地方忘清零了，我真是个sb

        for (const auto & armor : armors) {
            cv::Mat rvec, tvec;
            bool success = pnp_solver->solvePnP(armor, rvec, tvec);
            if (success) {
                // Fill basic info
                armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
                armor_msg.number = armor.number;

                // Fill pose
                armor_msg.pose.position.x = tvec.at<double>(0);
                armor_msg.pose.position.y = tvec.at<double>(1);
                armor_msg.pose.position.z = tvec.at<double>(2);
                // rvec to 3x3 rotation matrix
                cv::Mat rotation_matrix;
                cv::Rodrigues(rvec, rotation_matrix);
                // // rotation matrix to quaternion
                tf2::Matrix3x3 tf2_rotation_matrix(
                rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
                rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
                rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
                rotation_matrix.at<double>(2, 2));
                tf2::Quaternion tf2_q;
                tf2_rotation_matrix.getRotation(tf2_q);
                armor_msg.pose.orientation = tf2::toMsg(tf2_q);

                // Fill the distance to image center
                armor_msg.distance_to_image_center = pnp_solver->calculateDistanceToCenter(armor.center);

                armors_msg.armors.emplace_back(armor_msg);                
            } else {
                ROS_WARN("PnP failed");
            }
        }

        armors_pub.publish(armors_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}
