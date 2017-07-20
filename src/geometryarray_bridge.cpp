/**
Software License Agreement (BSD)

\file      geometryarray_bridge.cpp
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
#include <ark_bridge/GeometryArray.h>
#include <ark_bridge/Point.h>
#include <ark_bridge/Attribute.h>
#include <ark_bridge/GeometryFeature.h>
#include <autonomy_msgs/GeometryArray.h>
#include <geometry_msgs/Point.h>
#include <autonomy_msgs/Attribute.h>
#include <autonomy_msgs/GeometryFeature.h>
#include <stdlib.h>

ros::Publisher pub;
std::string lcm_topic, ros_topic, direction;

void rosCallback(const autonomy_msgs::GeometryArray::ConstPtr& msg)
{
  ark_bridge::GeometryArray bridge_message;

  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.frame_id = msg->header.frame_id;
  bridge_message.header.stamp = msg->header.stamp;

  bridge_message.nfeatures = msg->features.size();

  for(int f = 0; f < msg->features.size(); f++){
      ark_bridge::GeometryFeature gf;

      gf.id = msg->features[f].id;
      gf.type = msg->features[f].type;
      gf.npoints = msg->features[f].points.size();
      gf.nattributes = msg->features[f].attributes.size();
      gf.ndata = msg->features[f].data.size();

      for(int p = 0; p < msg->features[f].points.size(); p++){
        ark_bridge::Point pp;
        pp.x = msg->features[f].points[p].x;
        pp.y = msg->features[f].points[p].y;
        pp.z = msg->features[f].points[p].z;

        gf.points.push_back(pp);
      }

      for(int a = 0; a < msg->features[f].attributes.size(); a++){
        ark_bridge::Attribute aa;
        aa.name = msg->features[f].attributes[a].name;
        aa.offset = msg->features[f].attributes[a].offset;
        aa.datatype = msg->features[f].attributes[a].datatype;
        aa.length = msg->features[f].attributes[a].length;

        gf.attributes.push_back(aa);
      }

      for(int d = 0; d < msg->features[f].data.size(); d++){
        gf.data.push_back(msg->features[f].data[d]);
      }

    bridge_message.features.push_back(gf);
  }

  pub.publish(bridge_message);
}

void lcmCallback(const ark_bridge::GeometryArray::ConstPtr& msg)
{
  autonomy_msgs::GeometryArray bridge_message;

  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.frame_id = msg->header.frame_id;
  bridge_message.header.stamp = msg->header.stamp;

  for(int f = 0; f < msg->features.size(); f++){
      autonomy_msgs::GeometryFeature gf;

      gf.id = msg->features[f].id;
      gf.type = msg->features[f].type;

      for(int p = 0; p < msg->features[f].points.size(); p++){
        geometry_msgs::Point pp;
        pp.x = msg->features[f].points[p].x;
        pp.y = msg->features[f].points[p].y;
        pp.z = msg->features[f].points[p].z;

        gf.points.push_back(pp);
      }

      for(int a = 0; a < msg->features[f].attributes.size(); a++){
        autonomy_msgs::Attribute aa;
        aa.name = msg->features[f].attributes[a].name;
        aa.offset = msg->features[f].attributes[a].offset;
        aa.datatype = msg->features[f].attributes[a].datatype;
        aa.length = msg->features[f].attributes[a].length;

        gf.attributes.push_back(aa);
      }

      for(int d = 0; d < msg->features[f].data.size(); d++){
        gf.data.push_back(msg->features[f].data[d]);
      }

    bridge_message.features.push_back(gf);
  }

  pub.publish(bridge_message);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "geometryarray_remapper");
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
      pub = nh.advertise<ark_bridge::GeometryArray>(lcm_topic, 10);
      sub = nh.subscribe(ros_topic, 10, rosCallback);
    }
    else{
      pub = nh.advertise<autonomy_msgs::GeometryArray>(ros_topic, 10);
      sub = nh.subscribe(lcm_topic, 10, lcmCallback);
    }

    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;

}
