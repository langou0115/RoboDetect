#ifndef _CAMERANODE_HPP_
#define _CAMERANODE_HPP_
#include "MvCameraControl.h"
#include "MvErrorDefine.h"
#include "image_transport/image_transport.h"
#include "image_transport/publisher.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cstdint>
#include <thread>
#include <robotparams/dynamictoolsConfig.h>
#include <dynamic_reconfigure/server.h>

class CameraNode
{
public:
    CameraNode() = delete;
    CameraNode(ros::NodeHandle nh);
    ~CameraNode();

private:
    std::thread capture_thread;
    int nRet = MV_OK;
    void * camera_handle;
    image_transport::ImageTransport it;
    image_transport::Publisher camera_pub;
    sensor_msgs::Image img_msg;

    MV_IMAGE_BASIC_INFO img_info;
    MV_CC_PIXEL_CONVERT_PARAM convert_param;
    int fail_count;

    void work();
    // dynamic Server
    void paramsCallBack(robotparams::dynamictoolsConfig &config, uint32_t level);
    dynamic_reconfigure::Server<robotparams::dynamictoolsConfig> camera_server;

};

#endif // !_CAMERANODE_HPP_
