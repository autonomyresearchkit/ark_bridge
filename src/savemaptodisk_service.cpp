#include <ros/ros.h>
#include <ros/console.h>
#include <ark_bridge/Empty.h>
#include <ark_bridge/SaveMapToDiskCall.h>
#include <map_data_msgs/SaveMapToDisk.h>
#include <stdlib.h>

ros::Publisher pub;
ros::ServiceClient serv;
std::string call_topic, response_topic, service_name;

void rosCallback(const ark_bridge::SaveMapToDiskCall::ConstPtr& msg)
{
  map_data_msgs::SaveMapToDisk srv;
  srv.request.map_topic = msg->map_topic;
  srv.request.timeout.sec = msg->timeout.sec;
  srv.request.timeout.nsec = msg->timeout.nsec;
  srv.request.filename = msg->filename;

  if(serv.call(srv)){
    ark_bridge::Empty response_message;
    pub.publish(response_message);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "savemaptodisk_servicer");
  ros::NodeHandle nh("~");
  ros::Subscriber sub;

  if(nh.getParam("call_topic", call_topic) && nh.getParam("service_name", service_name) && nh.getParam("response_topic", response_topic)){
    ROS_INFO("(%s) --> <%s> --> (%s)", call_topic.c_str(), service_name.c_str(), response_topic.c_str());

    pub = nh.advertise<ark_bridge::Empty>(response_topic, 1, true);
    sub = nh.subscribe(call_topic, 10, rosCallback);
    serv = nh.serviceClient<map_data_msgs::SaveMapToDisk>(service_name);
    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;
}
