/**
Software License Agreement (BSD)

\file      pointcloud2_bridge.cpp
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
#include <ark_bridge/PointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <ark_bridge/PointField.h>
#include <sensor_msgs/PointField.h>
#include <stdlib.h>

ros::Publisher pub;
std::string lcm_topic, ros_topic, direction;

void rosCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  ark_bridge::PointCloud2 bridge_message;

  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.frame_id = msg->header.frame_id;
  bridge_message.header.stamp = msg->header.stamp;

  bridge_message.height = msg->height;
  bridge_message.width = msg->width;
  bridge_message.is_bigendian = msg->is_bigendian;
  bridge_message.point_step = msg->point_step;
  bridge_message.row_step = msg->row_step;
  bridge_message.is_dense = msg->is_dense;

  bridge_message.ndata = msg->data.size();
  for(int i = 0; i < msg->data.size(); i++){ 
    bridge_message.data.push_back(msg->data[i]);
  }

  bridge_message.nfields = msg->fields.size();

  for(int i = 0; i < msg->fields.size(); i++){
    ark_bridge::PointField pfield;

    pfield.name = msg->fields[i].name;
    pfield.offset = msg->fields[i].offset;
    pfield.datatype = msg->fields[i].datatype;
    pfield.count = msg->fields[i].count;

    bridge_message.fields.push_back(pfield);
  }

  pub.publish(bridge_message);
}

void lcmCallback(const ark_bridge::PointCloud2::ConstPtr& msg)
{
  sensor_msgs::PointCloud2 bridge_message;

  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.frame_id = msg->header.frame_id;
  bridge_message.header.stamp = msg->header.stamp;

  bridge_message.height = msg->height;
  bridge_message.width = msg->width;
  bridge_message.is_bigendian = msg->is_bigendian;
  bridge_message.point_step = msg->point_step;
  bridge_message.row_step = msg->row_step;
  bridge_message.is_dense = msg->is_dense;

  for(int i = 0; i < msg->data.size(); i++){ 
    bridge_message.data.push_back(msg->data[i]);
  }

  for(int i = 0; i < msg->fields.size(); i++){
    sensor_msgs::PointField pfield;

    pfield.name = msg->fields[i].name;
    pfield.offset = msg->fields[i].offset;
    pfield.datatype = msg->fields[i].datatype;
    pfield.count = msg->fields[i].count;

    bridge_message.fields.push_back(pfield);
  }

  pub.publish(bridge_message);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointcloud2_remapper");
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
      pub = nh.advertise<ark_bridge::PointCloud2>(lcm_topic, 10);
      sub = nh.subscribe(ros_topic, 10, rosCallback);
    }
    else{
      pub = nh.advertise<sensor_msgs::PointCloud2>(ros_topic, 10);
      sub = nh.subscribe(lcm_topic, 10, lcmCallback);
    }

    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;

}
