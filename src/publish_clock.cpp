#include <ros/ros.h>
#include <ros/console.h>
#include <rosgraph_msgs/Clock.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "clock_publisher");

  ros::NodeHandle n;
  ros::Publisher clock_pub = n.advertise<rosgraph_msgs::Clock>("clock", 5);

  ros::Rate loop_rate(50);

  while (ros::ok())
  {
    rosgraph_msgs::Clock msg;
    msg.clock = ros::Time::now();

    clock_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
