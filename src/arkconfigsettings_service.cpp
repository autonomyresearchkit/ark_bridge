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

  srv.request.max_fwd_velocity = msg->max_fwd_velocity;
  srv.request.max_rev_velocity = msg->max_rev_velocity;
  srv.request.min_fwd_velocity = msg->min_fwd_velocity;
  srv.request.max_accel = msg->max_accel;
  srv.request.max_decel = msg->max_decel;
  srv.request.max_ang_velocity = msg->max_ang_velocity;
  srv.request.max_ang_accel = msg->max_ang_accel;
  srv.request.max_lateral_accel = msg->max_lateral_accel;
  srv.request.vehicle_length = msg->vehicle_length;
  srv.request.vehicle_width = msg->vehicle_width;
  srv.request.stopping_distance_1M = msg->stopping_distance_1M;
  srv.request.lidar_spacing = msg->lidar_spacing;
  srv.request.laser_fov = msg->laser_fov;
  srv.request.vehicle_gear = msg->vehicle_gear;
  srv.request.curve_type = msg->curve_type;
  srv.request.goal_threshold = msg->goal_threshold;
  srv.request.orientation_corr_threshold = msg->orientation_corr_threshold;
  srv.request.mpc_horizon = msg->mpc_horizon;
  srv.request.min_lookahead = msg->min_lookahead;
  srv.request.max_lookahead = msg->max_lookahead;
  srv.request.horizon_percent_change = msg->horizon_percent_change;
  srv.request.lookahead_smoother = msg->lookahead_smoother;
  srv.request.lookahead_factor = msg->lookahead_factor;
  srv.request.curvature_slowdown = msg->curvature_slowdown;

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
