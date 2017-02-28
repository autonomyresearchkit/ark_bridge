#include <ros/ros.h>
#include <ros/console.h>
#include <lcm_to_ros/tfMessage.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <lcm_to_ros/TransformStamped.h>
#include <stdlib.h>

ros::Publisher pub;
std::string lcm_topic, ros_topic, direction;

void rosCallback(const tf::tfMessage::ConstPtr& msg)
{
  lcm_to_ros::tfMessage bridge_message;

  bridge_message.ntransforms = msg->transforms.size();

  for(int i = 0; i < msg->transforms.size(); i++){
    lcm_to_ros::TransformStamped tform;

    tform.child_frame_id = msg->transforms[i].child_frame_id;

    tform.header.seq = msg->transforms[i].header.seq;
    tform.header.frame_id = msg->transforms[i].header.frame_id;
    tform.header.stamp = msg->transforms[i].header.stamp;

    tform.transform.translation.x = msg->transforms[i].transform.translation.x;
    tform.transform.translation.y = msg->transforms[i].transform.translation.y;
    tform.transform.translation.z = msg->transforms[i].transform.translation.z;

    tform.transform.rotation.x = msg->transforms[i].transform.rotation.x;
    tform.transform.rotation.y = msg->transforms[i].transform.rotation.y;
    tform.transform.rotation.z = msg->transforms[i].transform.rotation.z;
    tform.transform.rotation.w = msg->transforms[i].transform.rotation.w;

    bridge_message.transforms.push_back(tform);
  }

  pub.publish(bridge_message);
}

void lcmCallback(const lcm_to_ros::tfMessage::ConstPtr& msg)
{
  tf::tfMessage bridge_message;

  for(int i = 0; i < msg->transforms.size(); i++){
    geometry_msgs::TransformStamped tform;

    tform.child_frame_id = msg->transforms[i].child_frame_id;

    tform.header.seq = msg->transforms[i].header.seq;
    tform.header.frame_id = msg->transforms[i].header.frame_id;
    tform.header.stamp = msg->transforms[i].header.stamp;

    tform.transform.translation.x = msg->transforms[i].transform.translation.x;
    tform.transform.translation.y = msg->transforms[i].transform.translation.y;
    tform.transform.translation.z = msg->transforms[i].transform.translation.z;

    tform.transform.rotation.x = msg->transforms[i].transform.rotation.x;
    tform.transform.rotation.y = msg->transforms[i].transform.rotation.y;
    tform.transform.rotation.z = msg->transforms[i].transform.rotation.z;
    tform.transform.rotation.w = msg->transforms[i].transform.rotation.w;

    bridge_message.transforms.push_back(tform);
  }

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
      pub = nh.advertise<lcm_to_ros::tfMessage>(lcm_topic, 10);
      sub = nh.subscribe(ros_topic, 10, rosCallback);
    }
    else{
      pub = nh.advertise<tf::tfMessage>(ros_topic, 10);
      sub = nh.subscribe(lcm_topic, 10, lcmCallback);
    }

    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;

}

