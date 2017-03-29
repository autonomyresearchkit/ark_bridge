#!/usr/bin/env python

import rospy
import time
from ark_bridge.msg import Mode, Status

def response_callback(data):
  global started
  if started:
    print data.status
    rospy.signal_shutdown("SLAM Mode Set")

def mode_setter():
  global started
  started = False
  print "Starting..."

  rospy.init_node("slam_setter")
  rospy.Subscriber("/ark_bridge/mode_manager_set_mode_response", Status, response_callback)
  time.sleep(0.3)
  started = True

  print "Setting SLAM"

  pub = rospy.Publisher("/ark_bridge/mode_manager_set_mode_call", Mode, latch=True, queue_size=1)
  msg = Mode()
  msg.mode = 1
  pub.publish(msg)

  rospy.spin()

  print "Done"

mode_setter()
