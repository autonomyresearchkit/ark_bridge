#!/usr/bin/env python
# Software License Agreement (BSD) 
#
# @author    Dave Niewinski <dniewinski@clearpathrobotics.com>
# @copyright (c) 2017, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#   following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#   following disclaimer in the documentation and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or
#   promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import time
from ark_bridge.msg import Empty, String

def response_callback(data):
  global started
  if started:
    rospy.signal_shutdown("Autonomy Paused")

def pauser():
  global started
  started = False
  print "Starting..."

  rospy.init_node("autonomy_pauser")
  rospy.Subscriber("/ark_bridge/control_selection_autonomy_pause_response", Empty, response_callback)
  time.sleep(0.3)
  started = True

  print "Pausing Autonomy"

  pub = rospy.Publisher("/ark_bridge/control_selection_autonomy_pause_call", String, latch=True, queue_size=1)
  pub.publish(String("Example"))

  rospy.spin()

  print "Done"

pauser()
