#include <ros/ros.h>
#include <ros/console.h>
#include <ark_bridge/PoseWithCovarianceStamped.h>
#include <ark_bridge/Bool.h>
#include <cpr_slam_msgs/SetPose.h>
#include <stdlib.h>

ros::Publisher pub;
ros::ServiceClient serv;
std::string call_topic, response_topic, service_name;

void rosCallback(const ark_bridge::PoseWithCovarianceStamped::ConstPtr& msg)
{
  cpr_slam_msgs::SetPose srv;

  srv.request.pose.header.seq = msg->header.seq;
  srv.request.pose.header.stamp = msg->header.stamp;
  srv.request.pose.header.frame_id = msg->header.frame_id;

  srv.request.pose.pose.pose.position.x = msg->pose.pose.position.x;
  srv.request.pose.pose.pose.position.y = msg->pose.pose.position.y;
  srv.request.pose.pose.pose.position.z = msg->pose.pose.position.z;

  srv.request.pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  srv.request.pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  srv.request.pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  srv.request.pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;

  srv.request.pose.pose.covariance = msg->pose.covariance;

  if(serv.call(srv)){
    ark_bridge::Bool response_message;
    response_message.data = srv.response.localized;

    pub.publish(response_message);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "setpose_servicer");
  ros::NodeHandle nh("~");
  ros::Subscriber sub;

  if(nh.getParam("call_topic", call_topic) && nh.getParam("service_name", service_name) && nh.getParam("response_topic", response_topic)){
    ROS_INFO("(%s) --> <%s> --> (%s)", call_topic.c_str(), service_name.c_str(), response_topic.c_str());

    pub = nh.advertise<ark_bridge::Bool>(response_topic, 1, true);
    sub = nh.subscribe(call_topic, 10, rosCallback);
    serv = nh.serviceClient<cpr_slam_msgs::SetPose>(service_name);
    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;
}
