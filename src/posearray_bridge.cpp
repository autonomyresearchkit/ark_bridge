#include <ros/ros.h>
#include <ros/console.h>
#include <lcm_to_ros/PoseArray.h>
#include <geometry_msgs/PoseArray.h>
#include <lcm_to_ros/PoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdlib.h>

ros::Publisher pub;
std::string lcm_topic, ros_topic, direction;

void rosCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  lcm_to_ros::PoseArray bridge_message;

  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.frame_id = msg->header.frame_id;
  bridge_message.header.stamp = msg->header.stamp;

  bridge_message.nposes = msg->poses.size();

  for(int i = 0; i < msg->poses.size(); i++){
    lcm_to_ros::Pose pstamp;

    pstamp.position.x = msg->poses[i].position.x;
    pstamp.position.y = msg->poses[i].position.y;
    pstamp.position.z = msg->poses[i].position.z;

    pstamp.orientation.x = msg->poses[i].orientation.x;
    pstamp.orientation.y = msg->poses[i].orientation.y;
    pstamp.orientation.z = msg->poses[i].orientation.z;
    pstamp.orientation.w = msg->poses[i].orientation.w;

    bridge_message.poses.push_back(pstamp);
  }

  pub.publish(bridge_message);
}

void lcmCallback(const lcm_to_ros::PoseArray::ConstPtr& msg)
{
  geometry_msgs::PoseArray bridge_message;

  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.frame_id = msg->header.frame_id;
  bridge_message.header.stamp = msg->header.stamp;

  for(int i = 0; i < msg->poses.size(); i++){
    geometry_msgs::Pose pstamp;

    pstamp.position.x = msg->poses[i].position.x;
    pstamp.position.y = msg->poses[i].position.y;
    pstamp.position.z = msg->poses[i].position.z;

    pstamp.orientation.x = msg->poses[i].orientation.x;
    pstamp.orientation.y = msg->poses[i].orientation.y;
    pstamp.orientation.z = msg->poses[i].orientation.z;
    pstamp.orientation.w = msg->poses[i].orientation.w;

    bridge_message.poses.push_back(pstamp);
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
      pub = nh.advertise<lcm_to_ros::PoseArray>(lcm_topic, 10);
      sub = nh.subscribe(ros_topic, 10, rosCallback);
    }
    else{
      pub = nh.advertise<geometry_msgs::PoseArray>(ros_topic, 10);
      sub = nh.subscribe(lcm_topic, 10, lcmCallback);
    }

    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;

}
