/**
Software License Agreement (BSD)

\file      tf_bridge.cpp
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
#include <ark_bridge/tfMessage.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <ark_bridge/TransformStamped.h>
#include <stdlib.h>

ros::Publisher pub;
std::string lcm_topic, ros_topic, direction;

void rosCallback(const tf::tfMessage::ConstPtr& msg)
{
  ark_bridge::tfMessage bridge_message;

  bridge_message.ntransforms = msg->transforms.size();

  for(int i = 0; i < msg->transforms.size(); i++){
    ark_bridge::TransformStamped tform;

    tform.child_frame_id = msg->transforms[i].child_frame_id;

    tform.header.seq = msg->transforms[i].header.seq;
    tform.header.frame_id = msg->transforms[i].header.frame_id;
    tform.header.stamp = msg->transforms[i].header.stamp;

    tform.transform.translation.x = msg->transforms[i].transform.translation.x;
    tform.transform.translation.y = msg->transforms[i].transform.translation.y;
    tform.transform.translation.z = msg->transforms[i].transform.translation.z;

    tform.transform.rotation.x = msg->transforms[i].transform.rotation.x;
    tform.transform.rotation.y = msg->transforms[i].transform.rotation.y;
    tform.transform.rotation.z = msg->transforms[i].transform.rotation.z;
    tform.transform.rotation.w = msg->transforms[i].transform.rotation.w;

    bridge_message.transforms.push_back(tform);
  }

  pub.publish(bridge_message);
}

void lcmCallback(const ark_bridge::tfMessage::ConstPtr& msg)
{
  tf::tfMessage bridge_message;

  for(int i = 0; i < msg->transforms.size(); i++){
    geometry_msgs::TransformStamped tform;

    tform.child_frame_id = msg->transforms[i].child_frame_id;

    tform.header.seq = msg->transforms[i].header.seq;
    tform.header.frame_id = msg->transforms[i].header.frame_id;
    tform.header.stamp = msg->transforms[i].header.stamp;

    tform.transform.translation.x = msg->transforms[i].transform.translation.x;
    tform.transform.translation.y = msg->transforms[i].transform.translation.y;
    tform.transform.translation.z = msg->transforms[i].transform.translation.z;

    tform.transform.rotation.x = msg->transforms[i].transform.rotation.x;
    tform.transform.rotation.y = msg->transforms[i].transform.rotation.y;
    tform.transform.rotation.z = msg->transforms[i].transform.rotation.z;
    tform.transform.rotation.w = msg->transforms[i].transform.rotation.w;

    bridge_message.transforms.push_back(tform);
  }

  pub.publish(bridge_message);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_remapper");
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
      pub = nh.advertise<ark_bridge::tfMessage>(lcm_topic, 10);
      sub = nh.subscribe(ros_topic, 10, rosCallback);
    }
    else{
      pub = nh.advertise<tf::tfMessage>(ros_topic, 10);
      sub = nh.subscribe(lcm_topic, 10, lcmCallback);
    }

    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;

}
