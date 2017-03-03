#include <ros/ros.h>
#include <ros/console.h>
#include <ark_bridge/Odometry.h>
#include <nav_msgs/Odometry.h>
#include <stdlib.h>

ros::Publisher pub;
std::string lcm_topic, ros_topic, direction;

void rosCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ark_bridge::Odometry bridge_message;

  bridge_message.child_frame_id = msg->child_frame_id;

  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.frame_id = msg->header.frame_id;
  bridge_message.header.stamp = msg->header.stamp;

  bridge_message.pose.pose.position.x = msg->pose.pose.position.x;
  bridge_message.pose.pose.position.y = msg->pose.pose.position.y;
  bridge_message.pose.pose.position.z = msg->pose.pose.position.z;

  bridge_message.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  bridge_message.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  bridge_message.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  bridge_message.pose.pose.orientation.w = msg->pose.pose.orientation.w;

  bridge_message.pose.covariance = msg->pose.covariance;

  bridge_message.twist.twist.linear.x = msg->twist.twist.linear.x;
  bridge_message.twist.twist.linear.y = msg->twist.twist.linear.y;
  bridge_message.twist.twist.linear.z = msg->twist.twist.linear.z;

  bridge_message.twist.twist.angular.x = msg->twist.twist.angular.x;
  bridge_message.twist.twist.angular.y = msg->twist.twist.angular.y;
  bridge_message.twist.twist.angular.z = msg->twist.twist.angular.z;

  bridge_message.twist.covariance = msg->twist.covariance;

  pub.publish(bridge_message);
}

void lcmCallback(const ark_bridge::Odometry::ConstPtr& msg)
{
  nav_msgs::Odometry bridge_message;

  bridge_message.child_frame_id = msg->child_frame_id;

  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.frame_id = msg->header.frame_id;
  bridge_message.header.stamp = msg->header.stamp;

  bridge_message.pose.pose.position.x = msg->pose.pose.position.x;
  bridge_message.pose.pose.position.y = msg->pose.pose.position.y;
  bridge_message.pose.pose.position.z = msg->pose.pose.position.z;

  bridge_message.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  bridge_message.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  bridge_message.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  bridge_message.pose.pose.orientation.w = msg->pose.pose.orientation.w;

  bridge_message.pose.covariance = msg->pose.covariance;

  bridge_message.twist.twist.linear.x = msg->twist.twist.linear.x;
  bridge_message.twist.twist.linear.y = msg->twist.twist.linear.y;
  bridge_message.twist.twist.linear.z = msg->twist.twist.linear.z;

  bridge_message.twist.twist.angular.x = msg->twist.twist.angular.x;
  bridge_message.twist.twist.angular.y = msg->twist.twist.angular.y;
  bridge_message.twist.twist.angular.z = msg->twist.twist.angular.z;

  bridge_message.twist.covariance = msg->twist.covariance;

  pub.publish(bridge_message);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_remapper");
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
      pub = nh.advertise<ark_bridge::Odometry>(lcm_topic, 10);
      sub = nh.subscribe(ros_topic, 10, rosCallback);
    }
    else{
      pub = nh.advertise<nav_msgs::Odometry>(ros_topic, 10);
      sub = nh.subscribe(lcm_topic, 10, lcmCallback);
    }

    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;

}
