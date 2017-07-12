#include <ros/ros.h>
#include <ros/console.h>
#include <ark_bridge/ArkConfigSettingsCall.h>
#include <ark_bridge/ArkConfigSettingsResponse.h>
#include <ark_configurator/ArkConfigSettings.h>
#include <stdlib.h>

ros::Publisher pub;
ros::ServiceClient serv;
std::string call_topic, response_topic, service_name;

void rosCallback(const ark_bridge::ArkConfigSettingsCall::ConstPtr& msg)
{
  ark_configurator::ArkConfigSettings srv;

  srv.request.max_fwd_linear_speed = msg->max_fwd_linear_speed;
	srv.request.max_rev_linear_speed = msg->max_rev_linear_speed;
	srv.request.min_linear_speed = msg->min_linear_speed;
	srv.request.max_linear_acceleration = msg->max_linear_acceleration;
	srv.request.max_linear_deceleration = msg->max_linear_deceleration;
	srv.request.max_ang_speed = msg->max_ang_speed;
	srv.request.max_ang_accel = msg->max_ang_accel;
	srv.request.max_lateral_accel = msg->max_lateral_accel;
	srv.request.vehicle_length = msg->vehicle_length;
	srv.request.vehicle_width = msg->vehicle_width;
	srv.request.stopping_distance_1M = msg->stopping_distance_1M;
	srv.request.lidar_spacing = msg->lidar_spacing;
	srv.request.laser_fov = msg->laser_fov;
	srv.request.goal_tolerance_xy = msg->goal_tolerance_xy;
	srv.request.goal_tolerance_yaw = msg->goal_tolerance_yaw;
	srv.request.camera_3d_memory = msg->camera_3d_memory;
	srv.request.drive_direction = msg->drive_direction;

  if(serv.call(srv)){
    ark_bridge::ArkConfigSettingsResponse response_message;

    response_message.result = srv.response.result;
    response_message.information = srv.response.information;

    pub.publish(response_message);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "arkconfigsettings_servicer");
  ros::NodeHandle nh("~");
  ros::Subscriber sub;

  if(nh.getParam("call_topic", call_topic) && nh.getParam("service_name", service_name) && nh.getParam("response_topic", response_topic)){
    ROS_INFO("(%s) --> <%s> --> (%s)", call_topic.c_str(), service_name.c_str(), response_topic.c_str());

    pub = nh.advertise<ark_bridge::ArkConfigSettingsResponse>(response_topic, 1, true);
    sub = nh.subscribe(call_topic, 10, rosCallback);
    serv = nh.serviceClient<ark_configurator::ArkConfigSettings>(service_name);
    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;
}
