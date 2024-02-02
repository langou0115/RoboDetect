#include "serial_port.h"
#include <iostream>
#include "ros/ros.h"  
#include "robotinterfaces/Target.h"
#include "serial_send.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <string>
#include <iostream>

// 接收到订阅的消息后，会进入消息回调函数
void callback(robotinterfaces::Target::ConstPtr target_msg)
{    
    cmd_vel_t send_data;

    //云台
    send_data.yaw = target_msg->yaw;
    send_data.pitch = 0;
    if(target_msg->tracking == false) send_data.shoot = 0;
    else send_data.shoot = 1;
    send_data.flag_gibal=0;
    
    //底盘
    send_data.chassis_vx=0;
    send_data.chassis_vy=0;
    send_data.chassis_wz=0;

    protocol_transmit(0x0101, (uint8_t*)&send_data, sizeof(cmd_vel_t));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ROS_INFO("\nSend date finished!\n");
}


int main (int argc, char** argv){

    ros::init(argc, argv, "sentry_send");
    cmd_vel_t send_data;
    ros::NodeHandle n;
    ROS_INFO("%d: %s", argc, argv[1]);  
    
    // 创建一个Subscriber，订阅名为data_chatter的topic，注册回调函数chatterCallback 
    ros::Subscriber sub = n.subscribe("target", 10, callback); 

    ros::spin();
}
