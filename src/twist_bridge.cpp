#include <ros/ros.h>
#include <ros/console.h>
#include <lcm_to_ros/Twist.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

ros::Publisher pub;
std::string lcm_topic, ros_topic, direction;

void rosCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  lcm_to_ros::Twist bridge_message;

  bridge_message.linear.x = msg->linear.x;
  bridge_message.linear.y = msg->linear.y;
  bridge_message.linear.z = msg->linear.z;

  bridge_message.angular.x = msg->angular.x;
  bridge_message.angular.y = msg->angular.y;
  bridge_message.angular.z = msg->angular.z;

  pub.publish(bridge_message);
}

void lcmCallback(const lcm_to_ros::Twist::ConstPtr& msg)
{
  geometry_msgs::Twist bridge_message;

  bridge_message.linear.x = msg->linear.x;
  bridge_message.linear.y = msg->linear.y;
  bridge_message.linear.z = msg->linear.z;

  bridge_message.angular.x = msg->angular.x;
  bridge_message.angular.y = msg->angular.y;
  bridge_message.angular.z = msg->angular.z;

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
      pub = nh.advertise<lcm_to_ros::Twist>(lcm_topic, 10);
      sub = nh.subscribe(ros_topic, 10, rosCallback);
    }
    else{
      pub = nh.advertise<geometry_msgs::Twist>(ros_topic, 10);
      sub = nh.subscribe(lcm_topic, 10, lcmCallback);
    }

    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;

}

