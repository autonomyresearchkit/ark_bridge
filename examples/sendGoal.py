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
import ark_bridge.msg
 
goalTopic = "/ark_bridge/strategy_manager_simple_goal"   # Goal Topic
statusTopic = "/ark_bridge/move_base_status"             # Status Topic
robotStatus = False     # Indicator if the robot has pending goals that have not been reached
 
jobIDs = []             # Record of currently accepted jobs
goal_publisher = None  
gotJobCallback = False  # Record if an initial state has been setup
 
def status_callback(data):
    global robotStatus, jobIDs, gotJobCallback
    newIDs = []
    newStatus = True
    for s in data.status_list:
        if s.status != 3:
            newStatus = False
        newIDs.append(s.goal_id.id)
    robotStatus = newStatus
    jobIDs = newIDs
    gotJobCallback = True
 
def orchestrator():
    global pos_sub, goal_publisher, positions, goalTopic
    print "Starting..."
    rospy.init_node("command_position")
 
    rospy.Subscriber(statusTopic, ark_bridge.msg.GoalStatusArray, status_callback)
    goal_publisher = rospy.Publisher(goalTopic, ark_bridge.msg.PoseStamped, latch=True, queue_size=1)
    
    print "Waiting for intial state..."
    while not gotJobCallback: # Loop, waiting for the job server to give an initial state
        time.sleep(0.1)
 
    if not rospy.is_shutdown():
        moveBase()     # Tell the robot to move to a position
 
def moveBase():
    global goal_publisher, robotStatus, jobIDs
    preMoveIDs = jobIDs # Save the current list of jobs
	
    print "Sending Goal"  
    position = ark_bridge.msg.PoseStamped()
    position.header.frame_id = "map"
    position.pose.position.x = 1.7
    position.pose.position.y = 0.0
    position.pose.position.z = 0.0
    position.pose.orientation.x = 0.0
    position.pose.orientation.y = 0.0
    position.pose.orientation.z = 0.0
    position.pose.orientation.w = 1.0
    position.header.stamp = rospy.Time.now()
     
    goal_publisher.publish(position)
    
    print "Waiting for Confirmation"     
    while cmp(preMoveIDs, jobIDs) == 0 and not rospy.is_shutdown(): # Wait until the job appears in the job server
        time.sleep(0.1)
 
    print "Waiting for motion to complete"
    while not robotStatus and not rospy.is_shutdown(): # Wait until all jobs have been marked as complete
        time.sleep(0.1)

    print "Motion Complete"

orchestrator()
