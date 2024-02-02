#include "CameraNode.hpp"

bool is_camera;
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Camera_Node");
    ros::NodeHandle nh;
    is_camera = nh.param("is_camera", true);

    CameraNode camera_node(nh);

    // 设置循环频率
    int delay; // ms
    ros::param::get("delay", delay);

    if (is_camera)
    {
        // 定时器回调函数
        auto timerCallback = [&camera_node](const ros::TimerEvent &)
        {
            camera_node.captureAndPublish();
        };

        ros::Timer timer = nh.createTimer(ros::Duration(delay / 1000.0), timerCallback);

        ros::spin(); // 开始事件循环
        return 0;
    }
    else
    {
        std::string video_path = ros::package::getPath("robotdetect") + "/imgs/Test.mp4";

        cv::VideoCapture cap(video_path);
        if (!cap.isOpened())
        {
            ROS_ERROR("Could not open video file.");
            return -1;
        }

        cv::Mat frame;
        sensor_msgs::ImagePtr msg;

        ros::Rate loop_rate(1000.0 / delay); // 设置和视频帧率相同的值

        while (ros::ok())
        {
            // 读取新的帧
            cap >> frame;
            if (frame.empty())
            {
                ROS_INFO("Video over.");
                break; // 如果视频播放完了，就退出循环
            }

            // 将OpenCV的图像转换成ROS图像消息
            std_msgs::Header header;
            header.frame_id = "camera";
            header.stamp = ros::Time::now();
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

            // 发布图像消息
            camera_node.image_pub.publish(msg);

            // 调用回调函数
            ros::spinOnce();

            // 按照设定的频率休眠
            loop_rate.sleep();
        }
    }
    return 0;
}

CameraNode::CameraNode(ros::NodeHandle nh) : it(nh)
{
    image_pub = it.advertise("camera_image", 1);
    if (is_camera)
    {
        fGainValue = nh.param("fGainValue", 10.0);
        exposure_time = nh.param("exposure_time", 5000);
        int nRet = MV_OK;

        // 枚举设备
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (nRet != MV_OK)
        {
            ROS_ERROR("MV_CC_EnumDevices failed, nRet [%x]", nRet);
            // 处理错误，可能需要抛出异常或结束程序
        }

        if (stDeviceList.nDeviceNum > 0)
        {
            void *handle;
            nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
            if (nRet != MV_OK)
            {
                ROS_ERROR("MV_CC_CreateHandle failed, nRet [%x]", nRet);
                // 处理错误
            }

            // 打开设备
            nRet = MV_CC_OpenDevice(handle);
            if (nRet != MV_OK)
            {
                ROS_ERROR("MV_CC_OpenDevice failed, nRet [%x]", nRet);
                // 处理错误
            }

            // 此处可添加更多相机配置代码，如设置曝光时间、增益等

            // 保存相机句柄供以后使用
            this->cameraHandle = handle;

            // 开始取流
            // start grab image
            nRet = MV_CC_StartGrabbing(handle);
            if (MV_OK != nRet)
            {
                ROS_ERROR("MV_CC_StartGrabbing fail! nRet [%x]", nRet);
            }
        }
        else
        {
            ROS_ERROR("No devices found.");
            is_camera = false;
            // 处理没有找到设备的情况
        }
    }
}

CameraNode::~CameraNode()
{
    if (this->cameraHandle && is_camera)
    {
        int nRet = MV_OK;

        // 停止取流
        nRet = MV_CC_StopGrabbing(this->cameraHandle);
        if (nRet != MV_OK)
        {
            ROS_ERROR("MV_CC_StopGrabbing failed, nRet [%x]", nRet);
        }

        // 关闭设备
        nRet = MV_CC_CloseDevice(this->cameraHandle);
        if (nRet != MV_OK)
        {
            ROS_ERROR("MV_CC_CloseDevice failed, nRet [%x]", nRet);
        }

        // 销毁句柄
        nRet = MV_CC_DestroyHandle(this->cameraHandle);
        if (nRet != MV_OK)
        {
            ROS_ERROR("MV_CC_DestroyHandle failed, nRet [%x]", nRet);
        }
    }
}

void CameraNode::captureAndPublish()
{
    int nRet = MV_OK;
    MV_FRAME_OUT stImageInfo = {0};

    // 设置增益
    nRet = MV_CC_SetFloatValue(this->cameraHandle, "Gain", fGainValue);
    if (nRet != MV_OK)
    {
        printf("Error: Set Gain failed [%x]\n", nRet);
    }

    nRet = MV_CC_SetFloatValue(this->cameraHandle, "ExposureTime", exposure_time);
    if (nRet != MV_OK)
    {
        printf("Error: Set ExposureTime failed [%x]\n", nRet);
    }

    // 相机图像捕获代码
    nRet = MV_CC_GetImageBuffer(this->cameraHandle, &stImageInfo, 1000);
    if (nRet != MV_OK)
    {
        ROS_ERROR("MV_CC_GetImageBuffer failed, nRet [%x]", nRet);
        // 处理错误，例如返回或重试
        return;
    }

    cv::Mat bayerImage(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC1, stImageInfo.pBufAddr);
    cv::Mat frame;

    // 将Bayer格式转换为BGR格式
    cv::cvtColor(bayerImage, frame, cv::COLOR_BayerRG2RGB);

    // 将OpenCV图像转换为ROS消息
    std_msgs::Header header;
    header.frame_id ="camera";
    header.stamp = ros::Time::now();
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

    // 发布图像消息
    image_pub.publish(msg);

    // 释放图像缓冲区
    nRet = MV_CC_FreeImageBuffer(this->cameraHandle, &stImageInfo);
    if (nRet != MV_OK)
    {
        ROS_ERROR("MV_CC_FreeImageBuffer failed, nRet [%x]", nRet);
    }
}
