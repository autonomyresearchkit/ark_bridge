/**
Software License Agreement (BSD)

\file      stepsequenceactionresult_bridge.cpp
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
#include <ark_bridge/StepSequenceActionResult.h>
#include <ark_bridge/Attribute.h>
#include <ark_bridge/Step.h>
#include <strategy_management_msgs/StepSequenceActionResult.h>
#include <autonomy_msgs/Attribute.h>
#include <autonomy_msgs/Step.h>

ros::Publisher pub;
std::string lcm_topic, ros_topic, direction;

void rosCallback(const strategy_management_msgs::StepSequenceActionResult::ConstPtr& msg)
{
  ark_bridge::StepSequenceActionResult bridge_message;

  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.stamp = msg->header.stamp;
  bridge_message.header.frame_id = msg->header.frame_id;

  bridge_message.status.goal_id.stamp = msg->status.goal_id.stamp;
  bridge_message.status.goal_id.id = msg->status.goal_id.id;

  bridge_message.status.status = msg->status.status;
  bridge_message.status.text = msg->status.text;

  bridge_message.result.nfinished_steps = msg->result.finished_steps.size();
  bridge_message.result.nunfinished_steps = msg->result.unfinished_steps.size();

  for(int fs = 0; fs < msg->result.finished_steps.size(); fs++){
    ark_bridge::Step step;
    step.id = msg->result.finished_steps[fs].id;
    step.type = msg->result.finished_steps[fs].type;
    step.interruptible = msg->result.finished_steps[fs].interruptible;
    step.nattributes = msg->result.finished_steps[fs].attributes.size();
    step.ndata = msg->result.finished_steps[fs].data.size();

    for(int a = 0; a < msg->result.finished_steps[fs].attributes.size(); a++){
      ark_bridge::Attribute at;

      at.name = msg->result.finished_steps[fs].attributes[a].name;
      at.offset = msg->result.finished_steps[fs].attributes[a].offset;
      at.datatype = msg->result.finished_steps[fs].attributes[a].datatype;
      at.length = msg->result.finished_steps[fs].attributes[a].length;

      step.attributes.push_back(at);
    }

    for(int d = 0; d < msg->result.finished_steps[fs].data.size(); d++){
      step.data.push_back(msg->result.finished_steps[fs].data[d]);
    }

    bridge_message.result.finished_steps.push_back(step);
  }

  for(int fs = 0; fs < msg->result.unfinished_steps.size(); fs++){
    ark_bridge::Step step;
    step.id = msg->result.unfinished_steps[fs].id;
    step.type = msg->result.unfinished_steps[fs].type;
    step.interruptible = msg->result.unfinished_steps[fs].interruptible;
    step.nattributes = msg->result.unfinished_steps[fs].attributes.size();
    step.ndata = msg->result.unfinished_steps[fs].data.size();

    for(int a = 0; a < msg->result.unfinished_steps[fs].attributes.size(); a++){
      ark_bridge::Attribute at;

      at.name = msg->result.unfinished_steps[fs].attributes[a].name;
      at.offset = msg->result.unfinished_steps[fs].attributes[a].offset;
      at.datatype = msg->result.unfinished_steps[fs].attributes[a].datatype;
      at.length = msg->result.unfinished_steps[fs].attributes[a].length;

      step.attributes.push_back(at);
    }

    for(int d = 0; d < msg->result.unfinished_steps[fs].data.size(); d++){
      step.data.push_back(msg->result.unfinished_steps[fs].data[d]);
    }

    bridge_message.result.unfinished_steps.push_back(step);
  }

  pub.publish(bridge_message);
}

void lcmCallback(const ark_bridge::StepSequenceActionResult::ConstPtr& msg)
{
  strategy_management_msgs::StepSequenceActionResult bridge_message;

  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.stamp = msg->header.stamp;
  bridge_message.header.frame_id = msg->header.frame_id;

  bridge_message.status.goal_id.stamp = msg->status.goal_id.stamp;
  bridge_message.status.goal_id.id = msg->status.goal_id.id;

  bridge_message.status.status = msg->status.status;
  bridge_message.status.text = msg->status.text;

  for(int fs = 0; fs < msg->result.finished_steps.size(); fs++){
    autonomy_msgs::Step step;
    step.id = msg->result.finished_steps[fs].id;
    step.type = msg->result.finished_steps[fs].type;
    step.interruptible = msg->result.finished_steps[fs].interruptible;

    for(int a = 0; a < msg->result.finished_steps[fs].attributes.size(); a++){
      autonomy_msgs::Attribute at;

      at.name = msg->result.finished_steps[fs].attributes[a].name;
      at.offset = msg->result.finished_steps[fs].attributes[a].offset;
      at.datatype = msg->result.finished_steps[fs].attributes[a].datatype;
      at.length = msg->result.finished_steps[fs].attributes[a].length;

      step.attributes.push_back(at);
    }

    for(int d = 0; d < msg->result.finished_steps[fs].data.size(); d++){
      step.data.push_back(msg->result.finished_steps[fs].data[d]);
    }

    bridge_message.result.finished_steps.push_back(step);
  }

  for(int fs = 0; fs < msg->result.unfinished_steps.size(); fs++){
    autonomy_msgs::Step step;
    step.id = msg->result.unfinished_steps[fs].id;
    step.type = msg->result.unfinished_steps[fs].type;
    step.interruptible = msg->result.unfinished_steps[fs].interruptible;

    for(int a = 0; a < msg->result.unfinished_steps[fs].attributes.size(); a++){
      autonomy_msgs::Attribute at;

      at.name = msg->result.unfinished_steps[fs].attributes[a].name;
      at.offset = msg->result.unfinished_steps[fs].attributes[a].offset;
      at.datatype = msg->result.unfinished_steps[fs].attributes[a].datatype;
      at.length = msg->result.unfinished_steps[fs].attributes[a].length;

      step.attributes.push_back(at);
    }

    for(int d = 0; d < msg->result.unfinished_steps[fs].data.size(); d++){
      step.data.push_back(msg->result.unfinished_steps[fs].data[d]);
    }

    bridge_message.result.unfinished_steps.push_back(step);
  }

  pub.publish(bridge_message);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "stepsequenceactionfeedback_remapper");
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
      pub = nh.advertise<ark_bridge::StepSequenceActionResult>(lcm_topic, 10, true);
      sub = nh.subscribe(ros_topic, 10, rosCallback);
    }
    else{
      pub = nh.advertise<strategy_management_msgs::StepSequenceActionResult>(ros_topic, 10, true);
      sub = nh.subscribe(lcm_topic, 10, lcmCallback);
    }

    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;

}
