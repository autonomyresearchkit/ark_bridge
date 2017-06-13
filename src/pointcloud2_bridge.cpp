#include <ros/ros.h>
#include <ros/console.h>
#include <ark_bridge/PointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <ark_bridge/PointField.h>
#include <sensor_msgs/PointField.h>
#include <stdlib.h>

ros::Publisher pub;
std::string lcm_topic, ros_topic, direction;

void rosCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  ark_bridge::PointCloud2 bridge_message;

  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.frame_id = msg->header.frame_id;
  bridge_message.header.stamp = msg->header.stamp;

  bridge_message.height = msg->height;
  bridge_message.width = msg->width;
  bridge_message.is_bigendian = msg->is_bigendian;
  bridge_message.point_step = msg->point_step;
  bridge_message.row_step = msg->row_step;
  bridge_message.is_dense = msg->is_dense;

  bridge_message.ndata = msg->data.size();
  for(int i = 0; i < msg->data.size(); i++){ 
    bridge_message.data.push_back(msg->data[i]);
  }

  bridge_message.nfields = msg->fields.size();

  for(int i = 0; i < msg->fields.size(); i++){
    ark_bridge::PointField pfield;

    pfield.name = msg->fields[i].name;
    pfield.offset = msg->fields[i].offset;
    pfield.datatype = msg->fields[i].datatype;
    pfield.count = msg->fields[i].count;

    bridge_message.fields.push_back(pfield);
  }

  pub.publish(bridge_message);
}

void lcmCallback(const ark_bridge::PointCloud2::ConstPtr& msg)
{
  sensor_msgs::PointCloud2 bridge_message;

  bridge_message.header.seq = msg->header.seq;
  bridge_message.header.frame_id = msg->header.frame_id;
  bridge_message.header.stamp = msg->header.stamp;

  bridge_message.height = msg->height;
  bridge_message.width = msg->width;
  bridge_message.is_bigendian = msg->is_bigendian;
  bridge_message.point_step = msg->point_step;
  bridge_message.row_step = msg->row_step;
  bridge_message.is_dense = msg->is_dense;

  for(int i = 0; i < msg->data.size(); i++){ 
    bridge_message.data.push_back(msg->data[i]);
  }

  for(int i = 0; i < msg->fields.size(); i++){
    sensor_msgs::PointField pfield;

    pfield.name = msg->fields[i].name;
    pfield.offset = msg->fields[i].offset;
    pfield.datatype = msg->fields[i].datatype;
    pfield.count = msg->fields[i].count;

    bridge_message.fields.push_back(pfield);
  }

  pub.publish(bridge_message);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointcloud2_remapper");
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
      pub = nh.advertise<ark_bridge::PointCloud2>(lcm_topic, 10);
      sub = nh.subscribe(ros_topic, 10, rosCallback);
    }
    else{
      pub = nh.advertise<sensor_msgs::PointCloud2>(ros_topic, 10);
      sub = nh.subscribe(lcm_topic, 10, lcmCallback);
    }

    ros::spin();
  }
  else{
    ROS_ERROR("Missing Parameters!");
  }

  return 0;

}
