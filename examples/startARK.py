#!/usr/bin/env python

import rospy
import time
from ark_bridge.msg import Empty

def response_callback(data):
  global started
  if started:
    rospy.signal_shutdown("ARK Started")

def ark_starter():
  global started
  started = False
  print "Starting..."

  rospy.init_node("ark_starter")
  rospy.Subscriber("/ark_bridge/ark_start_response", Empty, response_callback)
  time.sleep(0.3)
  started = True

  print "Starting ARK"

  pub = rospy.Publisher("/ark_bridge/ark_start_call", Empty, latch=True, queue_size=1)
  pub.publish(Empty())

  rospy.spin()

  print "Done"

ark_starter()
