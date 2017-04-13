#include <ros/ros.h>
#include <ros/console.h>
#include <ark_bridge/Clock.h>
#include <rosgraph_msgs/Clock.h>
#include <stdlib.h>

ros::Publisher pub;
std::string lcm_topic, ros_topic, direction;

void rosCallback(const rosgraph_msgs::Clock::ConstPtr& msg)
{
  ark_bridge::Clock bridge_message;

  bridge_message.clock = msg->clock;

  pub.publish(bridge_message);
}

void lcmCallback(const ark_bridge::Clock::ConstPtr& msg)
{
  rosgraph_msgs::Clock bridge_message;

  bridge_message.clock = msg->clock;

  pub.publish(bridge_message);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "clock_remapper");
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
      pub = nh.advertise<ark_bridge::Clock>(lcm_topic, 10);
      sub = nh.subscribe(ros_topic, 10, rosCallback);
    }
    else{
      pub = nh.advertise<rosgraph_msgs::Clock>(ros_topic, 10);
      sub = nh.subscribe(lcm_topic, 10, lcmCallback);
    }

    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;

}
