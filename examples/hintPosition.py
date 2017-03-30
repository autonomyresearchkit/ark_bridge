#!/usr/bin/env python

import rospy
import time
from ark_bridge.msg import PoseWithCovarianceStamped, Bool

def response_callback(data):
  global started
  if started:
    if data:
	print "Located!"
    else:
	print "Localization failed!"
    rospy.signal_shutdown("Autonomy Resumed")

def hint_position():
  global started
  started = False
  print "Starting..."

  rospy.init_node("position_hinter")
  rospy.Subscriber("/ark_bridge/slam_set_initial_pose_response", Bool, response_callback)
  time.sleep(0.3)
  started = True

  print "Hinting New Position"

  pub = rospy.Publisher("/ark_bridge/slam_set_initial_pose_call", PoseWithCovarianceStamped, latch=True, queue_size=1)
  pos = PoseWithCovarianceStamped()
  pos.header.frame_id="map"
  pos.pose.pose.position.y = 0.4
  pos.pose.pose.orientation.w = 1.0

  pub.publish(pos)

  rospy.spin()

  print "Done"

hint_position()
