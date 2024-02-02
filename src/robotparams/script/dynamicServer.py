#!/usr/bin/env python3

import rospy
from dynamic_reconfigure.server import Server
from robotparams.cfg import dynamictoolsConfig

# 将更改的参数同步到参数服务器中
def dynamic_reconfigure_callback(config, level):
    # Update rosparam values based on dynamic reconfigure
    # rospy.set_param("detect_color", config.detect_color)
    # rospy.set_param("threshold", config.threshold)
    # rospy.set_param("binary_thres", config.binary_thres)
    # rospy.set_param("DEBUG", config.DEBUG)

    return config

if __name__ == "__main__":
    rospy.init_node('dynamic_reconfig_to_rosparam_node')

    # Set up dynamic reconfigure server
    srv = Server(dynamictoolsConfig, dynamic_reconfigure_callback)

    rospy.spin()
