#include <ros/ros.h>
#include <ros/console.h>
#include <ark_bridge/ControlSelectionState.h>
#include <autonomy_msgs/ControlSelectionState.h>
#include <stdlib.h>

ros::Publisher pub;
std::string lcm_topic, ros_topic, direction;

void rosCallback(const autonomy_msgs::ControlSelectionState::ConstPtr& msg)
{
  ark_bridge::ControlSelectionState bridge_message;

  bridge_message.core.enabled = msg->core.enabled;
  bridge_message.core.paused = msg->core.paused;
  bridge_message.autonomy.enabled = msg->autonomy.enabled;
  bridge_message.autonomy.paused = msg->autonomy.paused;
  bridge_message.mode.mode = msg->mode.mode;

  pub.publish(bridge_message);
}

void lcmCallback(const ark_bridge::ControlSelectionState::ConstPtr& msg)
{
  autonomy_msgs::ControlSelectionState bridge_message;

  bridge_message.core.enabled = msg->core.enabled;
  bridge_message.core.paused = msg->core.paused;
  bridge_message.autonomy.enabled = msg->autonomy.enabled;
  bridge_message.autonomy.paused = msg->autonomy.paused;
  bridge_message.mode.mode = msg->mode.mode;

  pub.publish(bridge_message);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "controlselectionstate_remapper");
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
      pub = nh.advertise<ark_bridge::ControlSelectionState>(lcm_topic, 10);
      sub = nh.subscribe(ros_topic, 10, rosCallback);
    }
    else{
      pub = nh.advertise<autonomy_msgs::ControlSelectionState>(ros_topic, 10);
      sub = nh.subscribe(lcm_topic, 10, lcmCallback);
    }

    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;

}
