#include <ros/ros.h>
#include <ros/console.h>
#include <ark_bridge/GetCurrentMapResponse.h>
#include <ark_bridge/Empty.h>
#include <map_data_msgs/GetCurrentMap.h>
#include <stdlib.h>

ros::Publisher pub;
ros::ServiceClient serv;
std::string call_topic, response_topic, service_name;

void rosCallback(const ark_bridge::Empty::ConstPtr& msg)
{
  map_data_msgs::GetCurrentMap srv;

  if(serv.call(srv)){
    ark_bridge::GetCurrentMapResponse response_message;
    response_message.data = srv.response.data;
    response_message.ndata = srv.response.data.size()

    pub.publish(response_message);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "getcurrentmap_servicer");
  ros::NodeHandle nh("~");
  ros::Subscriber sub;

  if(nh.getParam("call_topic", call_topic) && nh.getParam("service_name", service_name) && nh.getParam("response_topic", response_topic)){
    ROS_INFO("(%s) --> <%s> --> (%s)", call_topic.c_str(), service_name.c_str(), response_topic.c_str());

    pub = nh.advertise<ark_bridge::GetCurrentMapResponse>(response_topic, 1, true);
    sub = nh.subscribe(call_topic, 10, rosCallback);
    serv = nh.serviceClient<map_data_msgs::GetCurrentMap>(service_name);
    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;
}
