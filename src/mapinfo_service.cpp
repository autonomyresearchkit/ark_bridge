#include <ros/ros.h>
#include <ros/console.h>
#include <ark_bridge/String.h>
#include <ark_bridge/MapInfoResponse.h>
#include <map_data_msgs/MapInfo.h>
#include <stdlib.h>

ros::Publisher pub;
ros::ServiceClient serv;
std::string call_topic, response_topic, service_name;

void rosCallback(const ark_bridge::String::ConstPtr& msg)
{
  map_data_msgs::MapInfo srv;

  srv.request.filename = msg->data;

  if(serv.call(srv)){
    ark_bridge::MapInfoResponse response_message;
    response_message.name = srv.response.name;
    response_message.md5sum = srv.response.md5sum;
    response_message.size = srv.response.size;

    pub.publish(response_message);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mapinfo_servicer");
  ros::NodeHandle nh("~");
  ros::Subscriber sub;

  if(nh.getParam("call_topic", call_topic) && nh.getParam("service_name", service_name) && nh.getParam("response_topic", response_topic)){
    ROS_INFO("(%s) --> <%s> --> (%s)", call_topic.c_str(), service_name.c_str(), response_topic.c_str());

    pub = nh.advertise<ark_bridge::MapInfoResponse>(response_topic, 1, true);
    sub = nh.subscribe(call_topic, 10, rosCallback);
    serv = nh.serviceClient<map_data_msgs::MapInfo>(service_name);
    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;
}
