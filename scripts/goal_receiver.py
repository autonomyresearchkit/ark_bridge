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

from std_msgs.msg import String
from ark_bridge.msg import Clock
from ark_bridge.msg import SendGoal
import time
import rospy
import json

#data: [{"id":"75411df4-5d74-4b44-a783-98e50b914ba5","interruptible":true,"type":"strategy_management/GoalMoveStep","attributes":{"target":{"position":{"x":-2.4282798767089844,"y":-10.475020408630371},"orientation":{"w":0.06833726307528165,"x":0,"y":0,"z":0.9976622767627228}},"position_tolerance":0.1,"orientation_tolerance":0.05}}]
data = {}
data["id"] = "NONE"
data["interruptible"] = True
data["type"] = "strategy_management/GoalMoveStep"
data["attributes"] = {}
data["attributes"]["target"] = {}
data["attributes"]["target"]["position"] = {}
data["attributes"]["target"]["position"]["x"] = 0.0
data["attributes"]["target"]["position"]["y"] = 0.0
data["attributes"]["target"]["orientation"] = {}
data["attributes"]["target"]["orientation"]["x"] = 0.0
data["attributes"]["target"]["orientation"]["y"] = 0.0
data["attributes"]["target"]["orientation"]["z"] = 0.0
data["attributes"]["target"]["orientation"]["w"] = 0.0
data["attributes"]["position_tolerance"] = 0.1
data["attributes"]["orientation_tolerance"] = 0.05
data2 = []
data2.append(data)

def goal_callback(msg):
  global pub, data2

  data["id"] = msg.id
  data["attributes"]["target"]["position"]["x"] = msg.pose.position.x
  data["attributes"]["target"]["position"]["y"] = msg.pose.position.y
  data["attributes"]["target"]["orientation"]["x"] = msg.pose.orientation.x
  data["attributes"]["target"]["orientation"]["y"] = msg.pose.orientation.y
  data["attributes"]["target"]["orientation"]["z"] = msg.pose.orientation.z
  data["attributes"]["target"]["orientation"]["w"] = msg.pose.orientation.w
  data["attributes"]["position_tolerance"] = msg.position_tolerance
  data["attributes"]["orientation_tolerance"] = msg.orientation_tolerance
  
  pubMsg = String()
  pubMsg.data = json.dumps(data2)
  pub.publish(pubMsg)

def parse_goal():
  global pub
  rospy.init_node("goal_receiver")
  pub = rospy.Publisher("/mapper/strategy_manager_proxy/send_goal", String, queue_size=1)
  rospy.Subscriber("/ark_bridge/send_goal", SendGoal, goal_callback)

  rospy.spin()

if __name__ == "__main__":
  parse_goal()
