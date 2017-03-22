#include <ros/ros.h>
#include <ros/console.h>
#include <ark_bridge/PauseControlCall.h>
#include <ark_bridge/PauseControlResponse.h>
#include <strategy_management_msgs/PauseControl.h>
#include <stdlib.h>

ros::Publisher pub;
ros::ServiceClient serv;
std::string call_topic, response_topic, service_name;

void rosCallback(const ark_bridge::PauseControlCall::ConstPtr& msg)
{
  strategy_management_msgs::PauseControl srv;

  srv.request.job_id = msg->job_id;
  srv.request.source = msg->source;

  if(serv.call(srv)){
    ark_bridge::PauseControlResponse response_message;
    response_message.result = srv.response.result;
    response_message.result_text = srv.response.result_text;

    response_message.status.status = srv.response.status.status;
    response_message.status.job_id = srv.response.status.job_id;
    response_message.status.sources = srv.response.status.sources;
    response_message.status.nsources = srv.response.status.sources.size();

    pub.publish(response_message);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pausecontrol_servicer");
  ros::NodeHandle nh("~");
  ros::Subscriber sub;

  if(nh.getParam("call_topic", call_topic) && nh.getParam("service_name", service_name) && nh.getParam("response_topic", response_topic)){
    ROS_INFO("(%s) --> <%s> --> (%s)", call_topic.c_str(), service_name.c_str(), response_topic.c_str());

    pub = nh.advertise<ark_bridge::PauseControlResponse>(response_topic, 1, true);
    sub = nh.subscribe(call_topic, 10, rosCallback);
    serv = nh.serviceClient<strategy_management_msgs::PauseControl>(service_name);
    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;
}
