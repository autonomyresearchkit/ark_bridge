#include <ros/ros.h>
#include <ros/console.h>
#include <ark_bridge/DistanceToGoal.h>
#include <strategy_management_msgs/DistanceToGoal.h>
#include <stdlib.h>

ros::Publisher pub;
std::string lcm_topic, ros_topic, direction;

void rosCallback(const strategy_management_msgs::DistanceToGoal::ConstPtr& msg)
{
  ark_bridge::DistanceToGoal bridge_message;

  bridge_message.job_id = msg->job_id;
  bridge_message.distance_to_goal = msg->distance_to_goal;

  pub.publish(bridge_message);
}

void lcmCallback(const ark_bridge::DistanceToGoal::ConstPtr& msg)
{
  strategy_management_msgs::DistanceToGoal bridge_message;
  bridge_message.distance_to_goal = msg->distance_to_goal;

  pub.publish(bridge_message);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "distancetogoal_remapper");
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
      pub = nh.advertise<ark_bridge::DistanceToGoal>(lcm_topic, 10);
      sub = nh.subscribe(ros_topic, 10, rosCallback);
    }
    else{
      pub = nh.advertise<strategy_management_msgs::DistanceToGoal>(ros_topic, 10);
      sub = nh.subscribe(lcm_topic, 10, lcmCallback);
    }

    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;

}
