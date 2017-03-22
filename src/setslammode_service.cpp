#include <ros/ros.h>
#include <ros/console.h>
#include <ark_bridge/Status.h>
#include <ark_bridge/Mode.h>
#include <cpr_slam_msgs/SetMode.h>
#include <stdlib.h>

ros::Publisher pub;
ros::ServiceClient serv;
std::string call_topic, response_topic, service_name;

void rosCallback(const ark_bridge::Mode::ConstPtr& msg)
{
  cpr_slam_msgs::SetMode srv;
  srv.request.mode.mode = msg->mode;

  if(serv.call(srv)){
    ark_bridge::Status response_message;
    response_message.status = srv.response.status;
    pub.publish(response_message);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "setslammode_servicer");
  ros::NodeHandle nh("~");
  ros::Subscriber sub;

  if(nh.getParam("call_topic", call_topic) && nh.getParam("service_name", service_name) && nh.getParam("response_topic", response_topic)){
    ROS_INFO("(%s) --> <%s> --> (%s)", call_topic.c_str(), service_name.c_str(), response_topic.c_str());

    pub = nh.advertise<ark_bridge::Status>(response_topic, 1, true);
    sub = nh.subscribe(call_topic, 10, rosCallback);
    serv = nh.serviceClient<cpr_slam_msgs::SetMode>(service_name);
    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;
}
