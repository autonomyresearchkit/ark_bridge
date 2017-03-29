#!/usr/bin/env python

import rospy
from ark_bridge.msg import PoseWithCovarianceStamped
 
def pos_sub_callback(data):
    position = data.pose.pose
    print "(X:" + str(position.position.x) + " Y:" + str(position.position.y) + " Z:" + str(position.position.z) + ")"
    print "(Q1:" + str(position.orientation.x) + " Q2:" + str(position.orientation.y) + " Q3:" + str(position.orientation.z) + " Q4:" + str(position.orientation.w) + ")"
 
def listener():
    rospy.init_node("position_monitor")
    rospy.Subscriber("ark_bridge/slam_estimated_pose", PoseWithCovarianceStamped, pos_sub_callback)
 
    rospy.spin()

listener()
