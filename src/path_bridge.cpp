/**
Software License Agreement (BSD)

\file      path_bridge.cpp
\authors   Dave Niewinski <dniewinski@clearpathrobotics.com>
\copyright Copyright (c) 2017, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the 
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <ros/ros.h>
#include <ros/console.h>
#include <ark_bridge/Path.h>
#include <nav_msgs/Path.h>
#include <ark_bridge/PoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdlib.h>

ros::Publisher pub;
std::string lcm_topic, ros_topic, direction;

void rosCallback(const nav_msgs::Path::ConstPtr& msg)
{
  ark_bridge::Path bridge_message;

  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.frame_id = msg->header.frame_id;
  bridge_message.header.stamp = msg->header.stamp;

  bridge_message.nposes = msg->poses.size();

  for(int i = 0; i < msg->poses.size(); i++){
    ark_bridge::PoseStamped pstamp;

    pstamp.header.seq = msg->poses[i].header.seq;
    pstamp.header.frame_id = msg->poses[i].header.frame_id;
    pstamp.header.stamp = msg->poses[i].header.stamp;

    pstamp.pose.position.x = msg->poses[i].pose.position.x;
    pstamp.pose.position.y = msg->poses[i].pose.position.y;
    pstamp.pose.position.z = msg->poses[i].pose.position.z;

    pstamp.pose.orientation.x = msg->poses[i].pose.orientation.x;
    pstamp.pose.orientation.y = msg->poses[i].pose.orientation.y;
    pstamp.pose.orientation.z = msg->poses[i].pose.orientation.z;
    pstamp.pose.orientation.w = msg->poses[i].pose.orientation.w;

    bridge_message.poses.push_back(pstamp);
  }

  pub.publish(bridge_message);
}

void lcmCallback(const ark_bridge::Path::ConstPtr& msg)
{
  nav_msgs::Path bridge_message;

  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.frame_id = msg->header.frame_id;
  bridge_message.header.stamp = msg->header.stamp;

  for(int i = 0; i < msg->poses.size(); i++){
    geometry_msgs::PoseStamped pstamp;

    pstamp.header.seq = msg->poses[i].header.seq;
    pstamp.header.frame_id = msg->poses[i].header.frame_id;
    pstamp.header.stamp = msg->poses[i].header.stamp;

    pstamp.pose.position.x = msg->poses[i].pose.position.x;
    pstamp.pose.position.y = msg->poses[i].pose.position.y;
    pstamp.pose.position.z = msg->poses[i].pose.position.z;

    pstamp.pose.orientation.x = msg->poses[i].pose.orientation.x;
    pstamp.pose.orientation.y = msg->poses[i].pose.orientation.y;
    pstamp.pose.orientation.z = msg->poses[i].pose.orientation.z;
    pstamp.pose.orientation.w = msg->poses[i].pose.orientation.w;

    bridge_message.poses.push_back(pstamp);
  }

  pub.publish(bridge_message);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_remapper");
  ros::NodeHandle nh("~");
  ros::Subscriber sub;

  if(nh.getParam("ros_topic", ros_topic) && nh.getParam("lcm_topic", lcm_topic) && nh.getParam("direction", direction)){
     if(!direction.compare("ros2lcm")){
       ROS_INFO("(%s) LCM <-- ROS (%s)", lcm_topic.c_str(), ros_topic.c_str());
    }
    else{
      ROS_INFO("(%s) LCM --> ROS (%s)", lcm_topic.c_str(), ros_topic.c_str());
    }

    if(!direction.compare("ros2lcm")){
      pub = nh.advertise<ark_bridge::Path>(lcm_topic, 10);
      sub = nh.subscribe(ros_topic, 10, rosCallback);
    }
    else{
      pub = nh.advertise<nav_msgs::Path>(ros_topic, 10);
      sub = nh.subscribe(lcm_topic, 10, lcmCallback);
    }

    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;

}
