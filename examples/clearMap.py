#!/usr/bin/env python

import rospy
import time
from ark_bridge.msg import Empty

def response_callback(data):
  global started
  if started:
    rospy.signal_shutdown("Map Cleared")

def map_clearer():
  global started
  started = False
  print "Starting..."

  rospy.init_node("map_clearer")
  rospy.Subscriber("/ark_bridge/map_data_clear_call", Empty, response_callback)
  time.sleep(0.3)
  started = True

  print "Clearing Map"

  pub = rospy.Publisher("/ark_bridge/map_data_clear_call", Empty, latch=True, queue_size=1)
  pub.publish(Empty())

  rospy.spin()

  print "Done"

map_clearer()
