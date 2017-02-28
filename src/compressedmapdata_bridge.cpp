#include <ros/ros.h>
#include <ros/console.h>
#include <lcm_to_ros/CompressedMapData.h>
#include <map_data_msgs/CompressedMapData.h>
#include <stdlib.h>

ros::Publisher pub;
std::string lcm_topic, ros_topic, direction;

void rosCallback(const map_data_msgs::CompressedMapData::ConstPtr& msg)
{
  lcm_to_ros::CompressedMapData bridge_message;

  bridge_message.ndata = msg->data.size()

  for(int i = 0; i < msg->data.size(); i++){
    bridge_message.data.push_back(msg->data[i]);
  }

  pub.publish(bridge_message);
}

void lcmCallback(const lcm_to_ros::CompressedMapData::ConstPtr& msg)
{
  map_data_msgs::CompressedMapData bridge_message;

  for(int i = 0; i < msg->transforms.size(); i++){
    geometry_msgs::TransformStamped tform;

    bridge_message.ndata = msg->data.size()

    for(int i = 0; i < msg->data.size(); i++){
      bridge_message.data.push_back(msg->data[i]);
    }

  pub.publish(bridge_message);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "compressedmapdata_remapper");
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
      pub = nh.advertise<lcm_to_ros::CompressedMapData>(lcm_topic, 10);
      sub = nh.subscribe(ros_topic, 10, rosCallback);
    }
    else{
      pub = nh.advertise<map_data_msgs::CompressedMapData>(ros_topic, 10);
      sub = nh.subscribe(lcm_topic, 10, lcmCallback);
    }

    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;

}
