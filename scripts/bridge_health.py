#!/usr/bin/env python

import rospy
import time
import math
from ark_bridge.msg import Clock

def echo_callback(data):
  now = rospy.get_rostime()
  delta = math.fabs((now - data.clock).to_sec() * 1000.0)

  if delta < 15:  
    rospy.logdebug("GOOD (" + str(delta) + " ms)")
  elif delta < 30:
    rospy.logdebug("SLOW (" + str(delta) + " ms)")
  else:
    rospy.logdebug("BAD (" + str(delta) + " ms)")

rospy.logdebug("Watching Bridge...")

rospy.init_node("echo_timer")
rospy.Subscriber("/ark_bridge/clock_echo", Clock, echo_callback)

rospy.spin()

