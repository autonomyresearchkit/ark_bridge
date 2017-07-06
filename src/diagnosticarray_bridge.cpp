#include <ros/ros.h>
#include <ros/console.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

#include <ark_bridge/DiagnosticArray.h>
#include <ark_bridge/DiagnosticStatus.h>
#include <ark_bridge/KeyValue.h>

#include <stdlib.h>

ros::Publisher pub;
std::string lcm_topic, ros_topic, direction;

void rosCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
{
  ark_bridge::DiagnosticArray bridge_message;

  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.frame_id = msg->header.frame_id;
  bridge_message.header.stamp = msg->header.stamp;

  bridge_message.nstatus = msg->status.size();
  for(int i = 0; i < msg->status.size(); i++){ 
    ark_bridge::DiagnosticStatus dstatus;

    dstatus.level = msg->status[i].level;
    dstatus.name = msg->status[i].name;
    dstatus.message = msg->status[i].message;
    dstatus.hardware_id = msg->status[i].hardware_id;

    dstatus.nvalues = msg->status[i].values.size();
    for(int ii = 0; ii < msg->status[i].values.size(); ii++){ 
      ark_bridge::KeyValue kvalue;

      kvalue.key = msg->status[i].values[ii].key;
      kvalue.value = msg->status[i].values[ii].value;

      dstatus.values.push_back(kvalue);
    }

    bridge_message.status.push_back(dstatus);
  }

  pub.publish(bridge_message);
}

void lcmCallback(const ark_bridge::DiagnosticArray::ConstPtr& msg)
{
  diagnostic_msgs::DiagnosticArray bridge_message;

  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.frame_id = msg->header.frame_id;
  bridge_message.header.stamp = msg->header.stamp;

  for(int i = 0; i < msg->status.size(); i++){ 
    diagnostic_msgs::DiagnosticStatus dstatus;

    dstatus.level = msg->status[i].level;
    dstatus.name = msg->status[i].name;
    dstatus.message = msg->status[i].message;
    dstatus.hardware_id = msg->status[i].hardware_id;

    for(int ii = 0; ii < msg->status[i].values.size(); ii++){ 
      diagnostic_msgs::KeyValue kvalue; 

      kvalue.key = msg->status[i].values[ii].key;
      kvalue.value = msg->status[i].values[ii].value;

      dstatus.values.push_back(kvalue);
    }

    bridge_message.status.push_back(dstatus);
  }

  pub.publish(bridge_message);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "diagnosticsarray_remapper");
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
      pub = nh.advertise<ark_bridge::DiagnosticArray>(lcm_topic, 10);
      sub = nh.subscribe(ros_topic, 10, rosCallback);
    }
    else{
      pub = nh.advertise<diagnostic_msgs::DiagnosticArray>(ros_topic, 10);
      sub = nh.subscribe(lcm_topic, 10, lcmCallback);
    }

    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;

}
