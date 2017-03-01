#include <ros/ros.h>
#include <ros/console.h>
#include <lcm_to_ros/LaserScan.h>
#include <sensor_msgs/LaserScan.h>
#include <stdlib.h>

ros::Publisher pub;
std::string lcm_topic, ros_topic, direction;

void rosCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  lcm_to_ros::LaserScan bridge_message;

  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.frame_id = msg->header.frame_id;
  bridge_message.header.stamp = msg->header.stamp;

  bridge_message.angle_min = msg->angle_min;
  bridge_message.angle_max = msg->angle_max;
  bridge_message.angle_increment = msg->angle_increment;
  bridge_message.time_increment = msg->time_increment;
  bridge_message.scan_time = msg->scan_time;
  bridge_message.range_min = msg->range_min;
  bridge_message.range_max = msg->range_max;

  bridge_message.nranges = msg->ranges.size();
  bridge_message.nintensities = msg->intensities.size();

  for(int j = 0; j < msg->ranges.size(); j++){
    bridge_message.ranges.push_back(msg->ranges[j]);
  }

  for(int j = 0; j < msg->intensities.size(); j++){
    bridge_message.intensities.push_back(msg->intensities[j]);
  }

  pub.publish(bridge_message);
}

void lcmCallback(const lcm_to_ros::LaserScan::ConstPtr& msg)
{
  sensor_msgs::LaserScan bridge_message;

  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.frame_id = msg->header.frame_id;
  bridge_message.header.stamp = msg->header.stamp;

  bridge_message.angle_min = msg->angle_min;
  bridge_message.angle_max = msg->angle_max;
  bridge_message.angle_increment = msg->angle_increment;
  bridge_message.time_increment = msg->time_increment;
  bridge_message.scan_time = msg->scan_time;
  bridge_message.range_min = msg->range_min;
  bridge_message.range_max = msg->range_max;

  for(int j = 0; j < msg->ranges.size(); j++){
    bridge_message.ranges.push_back(msg->ranges[j]);
  }

  for(int j = 0; j < msg->intensities.size(); j++){
    bridge_message.intensities.push_back(msg->intensities[j]);
  }

  pub.publish(bridge_message);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "laserscan_remapper");
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
      pub = nh.advertise<lcm_to_ros::LaserScan>(lcm_topic, 10);
      sub = nh.subscribe(ros_topic, 10, rosCallback);
    }
    else{
      pub = nh.advertise<sensor_msgs::LaserScan>(ros_topic, 10);
      sub = nh.subscribe(lcm_topic, 10, lcmCallback);
    }

    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;

}
