/**
Software License Agreement (BSD)

\file      stepsequenceactionfeedback_bridge.cpp
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
#include <ark_bridge/StepSequenceActionFeedback.h>
#include <ark_bridge/Attribute.h>
#include <ark_bridge/Step.h>
#include <strategy_management_msgs/StepSequenceActionFeedback.h>
#include <autonomy_msgs/Attribute.h>
#include <autonomy_msgs/Step.h>
#include <stdlib.h>

ros::Publisher pub;
std::string lcm_topic, ros_topic, direction;

void rosCallback(const strategy_management_msgs::StepSequenceActionFeedback::ConstPtr& msg)
{
  ark_bridge::StepSequenceActionFeedback bridge_message;

  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.stamp = msg->header.stamp;
  bridge_message.header.frame_id = msg->header.frame_id;

  bridge_message.status.goal_id.stamp = msg->status.goal_id.stamp;
  bridge_message.status.goal_id.id = msg->status.goal_id.id;

  bridge_message.status.status = msg->status.status;
  bridge_message.status.text = msg->status.text;

  bridge_message.feedback.nfinished_steps = msg->feedback.finished_steps.size();
  bridge_message.feedback.nremaining_steps = msg->feedback.remaining_steps.size();

  for(int fs = 0; fs < msg->feedback.finished_steps.size(); fs++){
    ark_bridge::Step step;
    step.id = msg->feedback.finished_steps[fs].id;
    step.type = msg->feedback.finished_steps[fs].type;
    step.interruptible = msg->feedback.finished_steps[fs].interruptible;
    step.nattributes = msg->feedback.finished_steps[fs].attributes.size();
    step.ndata = msg->feedback.finished_steps[fs].data.size();

    for(int a = 0; a < msg->feedback.finished_steps[fs].attributes.size(); a++){
      ark_bridge::Attribute at;

      at.name = msg->feedback.finished_steps[fs].attributes[a].name;
      at.offset = msg->feedback.finished_steps[fs].attributes[a].offset;
      at.datatype = msg->feedback.finished_steps[fs].attributes[a].datatype;
      at.length = msg->feedback.finished_steps[fs].attributes[a].length;

      step.attributes.push_back(at);
    }

    for(int d = 0; d < msg->feedback.finished_steps[fs].data.size(); d++){
      step.data.push_back(msg->feedback.finished_steps[fs].data[d]);
    }

    bridge_message.feedback.finished_steps.push_back(step);
  }

  bridge_message.feedback.current_step.id = msg->feedback.current_step.id;
  bridge_message.feedback.current_step.type = msg->feedback.current_step.type;
  bridge_message.feedback.current_step.interruptible = msg->feedback.current_step.interruptible;
  bridge_message.feedback.current_step.nattributes = msg->feedback.current_step.attributes.size();
  bridge_message.feedback.current_step.ndata = msg->feedback.current_step.data.size();

  for(int a = 0; a < msg->feedback.current_step.attributes.size(); a++){
    ark_bridge::Attribute at;

    at.name = msg->feedback.current_step.attributes[a].name;
    at.offset = msg->feedback.current_step.attributes[a].offset;
    at.datatype = msg->feedback.current_step.attributes[a].datatype;
    at.length = msg->feedback.current_step.attributes[a].length;

    bridge_message.feedback.current_step.attributes.push_back(at);
  }

  for(int d = 0; d < msg->feedback.current_step.data.size(); d++){
    bridge_message.feedback.current_step.data.push_back(msg->feedback.current_step.data[d]);
  }

  for(int fs = 0; fs < msg->feedback.remaining_steps.size(); fs++){
    ark_bridge::Step step;
    step.id = msg->feedback.remaining_steps[fs].id;
    step.type = msg->feedback.remaining_steps[fs].type;
    step.interruptible = msg->feedback.remaining_steps[fs].interruptible;
    step.nattributes = msg->feedback.remaining_steps[fs].attributes.size();
    step.ndata = msg->feedback.remaining_steps[fs].data.size();

    for(int a = 0; a < msg->feedback.remaining_steps[fs].attributes.size(); a++){
      ark_bridge::Attribute at;

      at.name = msg->feedback.remaining_steps[fs].attributes[a].name;
      at.offset = msg->feedback.remaining_steps[fs].attributes[a].offset;
      at.datatype = msg->feedback.remaining_steps[fs].attributes[a].datatype;
      at.length = msg->feedback.remaining_steps[fs].attributes[a].length;

      step.attributes.push_back(at);
    }

    for(int d = 0; d < msg->feedback.remaining_steps[fs].data.size(); d++){
      step.data.push_back(msg->feedback.remaining_steps[fs].data[d]);
    }

    bridge_message.feedback.remaining_steps.push_back(step);
  }

  bridge_message.feedback.current_step_progress = msg->feedback.current_step_progress;
  bridge_message.feedback.total_progress = msg->feedback.total_progress;

  pub.publish(bridge_message);
}

void lcmCallback(const ark_bridge::StepSequenceActionFeedback::ConstPtr& msg)
{
  strategy_management_msgs::StepSequenceActionFeedback bridge_message;

  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.stamp = msg->header.stamp;
  bridge_message.header.frame_id = msg->header.frame_id;

  bridge_message.status.goal_id.stamp = msg->status.goal_id.stamp;
  bridge_message.status.goal_id.id = msg->status.goal_id.id;

  bridge_message.status.status = msg->status.status;
  bridge_message.status.text = msg->status.text;

  for(int fs = 0; fs < msg->feedback.finished_steps.size(); fs++){
    autonomy_msgs::Step step;
    step.id = msg->feedback.finished_steps[fs].id;
    step.type = msg->feedback.finished_steps[fs].type;
    step.interruptible = msg->feedback.finished_steps[fs].interruptible;

    for(int a = 0; a < msg->feedback.finished_steps[fs].attributes.size(); a++){
      autonomy_msgs::Attribute at;

      at.name = msg->feedback.finished_steps[fs].attributes[a].name;
      at.offset = msg->feedback.finished_steps[fs].attributes[a].offset;
      at.datatype = msg->feedback.finished_steps[fs].attributes[a].datatype;
      at.length = msg->feedback.finished_steps[fs].attributes[a].length;

      step.attributes.push_back(at);
    }

    for(int d = 0; d < msg->feedback.finished_steps[fs].data.size(); d++){
      step.data.push_back(msg->feedback.finished_steps[fs].data[d]);
    }

    bridge_message.feedback.finished_steps.push_back(step);
  }

  bridge_message.feedback.current_step.id = msg->feedback.current_step.id;
  bridge_message.feedback.current_step.type = msg->feedback.current_step.type;
  bridge_message.feedback.current_step.interruptible = msg->feedback.current_step.interruptible;

  for(int a = 0; a < msg->feedback.current_step.attributes.size(); a++){
    autonomy_msgs::Attribute at;

    at.name = msg->feedback.current_step.attributes[a].name;
    at.offset = msg->feedback.current_step.attributes[a].offset;
    at.datatype = msg->feedback.current_step.attributes[a].datatype;
    at.length = msg->feedback.current_step.attributes[a].length;

    bridge_message.feedback.current_step.attributes.push_back(at);
  }

  for(int d = 0; d < msg->feedback.current_step.data.size(); d++){
    bridge_message.feedback.current_step.data.push_back(msg->feedback.current_step.data[d]);
  }

  for(int fs = 0; fs < msg->feedback.remaining_steps.size(); fs++){
    autonomy_msgs::Step step;
    step.id = msg->feedback.remaining_steps[fs].id;
    step.type = msg->feedback.remaining_steps[fs].type;
    step.interruptible = msg->feedback.remaining_steps[fs].interruptible;

    for(int a = 0; a < msg->feedback.remaining_steps[fs].attributes.size(); a++){
      autonomy_msgs::Attribute at;

      at.name = msg->feedback.remaining_steps[fs].attributes[a].name;
      at.offset = msg->feedback.remaining_steps[fs].attributes[a].offset;
      at.datatype = msg->feedback.remaining_steps[fs].attributes[a].datatype;
      at.length = msg->feedback.remaining_steps[fs].attributes[a].length;

      step.attributes.push_back(at);
    }

    for(int d = 0; d < msg->feedback.remaining_steps[fs].data.size(); d++){
      step.data.push_back(msg->feedback.remaining_steps[fs].data[d]);
    }

    bridge_message.feedback.remaining_steps.push_back(step);
  }

  bridge_message.feedback.current_step_progress = msg->feedback.current_step_progress;
  bridge_message.feedback.total_progress = msg->feedback.total_progress;

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
      pub = nh.advertise<ark_bridge::StepSequenceActionFeedback>(lcm_topic, 10, true);
      sub = nh.subscribe(ros_topic, 10, rosCallback);
    }
    else{
      pub = nh.advertise<strategy_management_msgs::StepSequenceActionFeedback>(ros_topic, 10, true);
      sub = nh.subscribe(lcm_topic, 10, lcmCallback);
    }

    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;

}
