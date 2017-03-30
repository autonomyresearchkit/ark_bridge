#!/usr/bin/env python

import rospy
import time
from ark_bridge.msg import Empty, String

def response_callback(data):
  global started
  if started:
    rospy.signal_shutdown("Autonomy Resumed")

def resumer():
  global started
  started = False
  print "Starting..."

  rospy.init_node("autonomy_resumer")
  rospy.Subscriber("/ark_bridge/control_selection_autonomy_resume_response", Empty, response_callback)
  time.sleep(0.3)
  started = True

  print "Resuming Autonomy"

  pub = rospy.Publisher("/ark_bridge/control_selection_autonomy_resume_call", String, latch=True, queue_size=1)
  pub.publish(String("Example"))

  rospy.spin()

  print "Done"

resumer()
