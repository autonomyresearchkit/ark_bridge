#include <ros/ros.h>
#include <ros/console.h>
#include <ark_bridge/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <ark_bridge/GoalStatus.h>
#include <actionlib_msgs/GoalStatus.h>
#include <stdlib.h>

ros::Publisher pub;
std::string lcm_topic, ros_topic, direction;

void rosCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
  ark_bridge::GoalStatusArray bridge_message;

  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.frame_id = msg->header.frame_id;
  bridge_message.header.stamp = msg->header.stamp;

  bridge_message.nstatus_list = msg->status_list.size();

  for(int i = 0; i < msg->status_list.size(); i++){
    ark_bridge::GoalStatus gStatus;

    gStatus.text = msg->status_list[i].text;
    gStatus.status = msg->status_list[i].status;

    gStatus.goal_id.stamp = msg->status_list[i].goal_id.stamp;
    gStatus.goal_id.id = msg->status_list[i].goal_id.id;

    bridge_message.status_list.push_back(gStatus);
  }

  pub.publish(bridge_message);
}

void lcmCallback(const ark_bridge::GoalStatusArray::ConstPtr& msg)
{
  actionlib_msgs::GoalStatusArray bridge_message;

  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.frame_id = msg->header.frame_id;
  bridge_message.header.stamp = msg->header.stamp;

  for(int i = 0; i < msg->status_list.size(); i++){
    actionlib_msgs::GoalStatus gStatus;

    gStatus.text = msg->status_list[i].text;
    gStatus.status = msg->status_list[i].status;

    gStatus.goal_id.stamp = msg->status_list[i].goal_id.stamp;
    gStatus.goal_id.id = msg->status_list[i].goal_id.id;

    bridge_message.status_list.push_back(gStatus);
  }

  pub.publish(bridge_message);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "posearray_remapper");
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
      pub = nh.advertise<ark_bridge::GoalStatusArray>(lcm_topic, 10);
      sub = nh.subscribe(ros_topic, 10, rosCallback);
    }
    else{
      pub = nh.advertise<actionlib_msgs::GoalStatusArray>(ros_topic, 10);
      sub = nh.subscribe(lcm_topic, 10, lcmCallback);
    }

    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;

}
