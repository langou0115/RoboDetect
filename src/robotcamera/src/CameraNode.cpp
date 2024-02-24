#include "CameraNode.hpp"
#include "MvCameraControl.h"
#include "MvErrorDefine.h"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "robot_camera");
    ros::NodeHandle nh;

    CameraNode camera_node(nh);
    return 0;
}

CameraNode::CameraNode(ros::NodeHandle nh):
    it(nh)
{
    ROS_INFO("Start Camera Node");
    MV_CC_DEVICE_INFO_LIST device_list;
    // enum device
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    ROS_INFO("Found camera count = %d", device_list.nDeviceNum);

    while (device_list.nDeviceNum == 0 && ros::ok()) {
        ROS_ERROR("No camera found!");
        ROS_INFO("Enum state: [%x]", nRet);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    }

    MV_CC_CreateHandle(&camera_handle, device_list.pDeviceInfo[0]);
    MV_CC_OpenDevice(camera_handle);
    
    // Get camera infomation
    MV_CC_GetImageInfo(camera_handle, &img_info);
    img_msg.data.reserve(img_info.nHeightMax * img_info.nWidthMax * 3);

    // Init convert param
    convert_param.nWidth = img_info.nWidthValue;
    convert_param.nHeight = img_info.nHeightValue;
    convert_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

    camera_pub = it.advertise("camera_image", 1);

    // 设置增益
    nRet = MV_CC_SetFloatValue(camera_handle, "Gain", 10.0);
    if (nRet != MV_OK)
    {
        printf("Error: Set Gain failed [%x]\n", nRet);
    }

    nRet = MV_CC_SetFloatValue(camera_handle, "ExposureTime", 10000);
    if (nRet != MV_OK)
    {
        printf("Error: Set ExposureTime failed [%x]\n", nRet);
    }

    MV_CC_StartGrabbing(camera_handle);

    camera_server.setCallback(boost::bind(&CameraNode::paramsCallBack, this, _1, _2));
    capture_thread = std::thread(&CameraNode::work, this);

    ros::spin();
}

void CameraNode::work()
{
    MV_FRAME_OUT out_frame;
    img_msg.header.frame_id = "camera";
    img_msg.encoding = "bgr8";
    while(ros::ok())
    {
        nRet = MV_CC_GetImageBuffer(camera_handle, &out_frame, 1000);
        if(MV_OK == nRet)
        {
            convert_param.pDstBuffer = img_msg.data.data();
            convert_param.nDstBufferSize = img_msg.data.size();
            convert_param.pSrcData = out_frame.pBufAddr;
            convert_param.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
            convert_param.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

            MV_CC_ConvertPixelType(camera_handle, &convert_param);
            img_msg.header.stamp = ros::Time::now();
            img_msg.height = out_frame.stFrameInfo.nHeight;
            img_msg.width = out_frame.stFrameInfo.nWidth;
            img_msg.step = out_frame.stFrameInfo.nWidth * 3;
            img_msg.data.resize(img_msg.width * img_msg.height * 3);

            camera_pub.publish(img_msg);
            MV_CC_FreeImageBuffer(camera_handle, &out_frame);
            fail_count = 0;
        }else{
            ROS_WARN("Get buffer failed! nRet: [%x]", nRet);
            MV_CC_StopGrabbing(camera_handle);
            MV_CC_StartGrabbing(camera_handle);
            fail_count++;
        }
        if(fail_count > 5){
            ROS_ERROR("Camera Failed");
            ros::shutdown();
        }
    }
}

void CameraNode::paramsCallBack(robotparams::dynamictoolsConfig &config, uint32_t level)
{
    // 设置增益
    nRet = MV_CC_SetFloatValue(camera_handle, "Gain", config.Gain);
    if(MV_OK != nRet){
        ROS_ERROR("Set Gain Failed");
    }

    // 设置曝光时间
    nRet = MV_CC_SetFloatValue(camera_handle, "ExposureTime", config.ExposureTime);
    if(MV_OK != nRet){
        ROS_ERROR("Set Exposure Time Failed");
    }
}

CameraNode::~CameraNode()
{
    if(capture_thread.joinable()){
        capture_thread.join();
    }
    if (camera_handle) {
      MV_CC_StopGrabbing(camera_handle);
      MV_CC_CloseDevice(camera_handle);
      MV_CC_DestroyHandle(&camera_handle);
    }
}