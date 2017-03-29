#!/usr/bin/env python

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
