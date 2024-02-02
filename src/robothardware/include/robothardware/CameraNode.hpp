#ifndef _CAMERANODE_HPP_
#define _CAMERANODE_HPP_

#include <iostream>
#include "ros/ros.h"
#include "ros/package.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <X11/Xlib.h>
#include <assert.h>
#include "math.h"
#include "MvCameraControl.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

class CameraNode
{
private:
    float fGainValue;
    float exposure_time;
    image_transport::ImageTransport it;
    void* cameraHandle; // 成员变量用于存储相机句柄
public:
    image_transport::Publisher image_pub;
    void captureAndPublish();
    CameraNode() = delete;
    CameraNode(ros::NodeHandle nh);
    ~CameraNode();
};


#endif // !_CAMERA_HPP_