#include <ros/ros.h>
#include <ros/console.h>
#include <lcm_to_ros/PoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdlib.h>

ros::Publisher pub;
std::string lcm_topic, ros_topic, direction;

void rosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  lcm_to_ros::PoseStamped bridge_message;

  bridge_message.seq = msg->seq;
  bridge_message.stamp = msg->stamp;
  bridge_message.frame_id = msg->frame_id;

  bridge_message.pose.posotion.x = msg->.pose.posotion.x;
  bridge_message.pose.posotion.y = msg->.pose.posotion.y;
  bridge_message.pose.posotion.z = msg->.pose.posotion.z;

  bridge_message.pose.orientation.x = msg->.pose.orientation.x;
  bridge_message.pose.orientation.y = msg->.pose.orientation.y;
  bridge_message.pose.orientation.z = msg->.pose.orientation.z;
  bridge_message.pose.orientation.w = msg->.pose.orientation.w;

  pub.publish(bridge_message);
}

void lcmCallback(const lcm_to_ros::PoseStamped::ConstPtr& msg)
{
  geometry_msgs::PoseStamped bridge_message;

  bridge_message.seq = msg->seq;
  bridge_message.stamp = msg->stamp;
  bridge_message.frame_id = msg->frame_id;

  bridge_message.pose.posotion.x = msg->.pose.posotion.x;
  bridge_message.pose.posotion.y = msg->.pose.posotion.y;
  bridge_message.pose.posotion.z = msg->.pose.posotion.z;

  bridge_message.pose.orientation.x = msg->.pose.orientation.x;
  bridge_message.pose.orientation.y = msg->.pose.orientation.y;
  bridge_message.pose.orientation.z = msg->.pose.orientation.z;
  bridge_message.pose.orientation.w = msg->.pose.orientation.w;

  pub.publish(bridge_message);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "posestamped_remapper");
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
      pub = nh.advertise<lcm_to_ros::PoseStamped>(lcm_topic, 10);
      sub = nh.subscribe(ros_topic, 10, rosCallback);
    }
    else{
      pub = nh.advertise<geometry_msgs::PoseStamped>(ros_topic, 10);
      sub = nh.subscribe(lcm_topic, 10, lcmCallback);
    }

    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;

}
