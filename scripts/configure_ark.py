#!/usr/bin/env python

from ark_bridge.msg import ArkConfigSettingsResponse, ArkConfigSettingsCall
import time
import rospy
import yaml
import sys

def response_callback(data):
  global started
  if started:
    print data.information
    rospy.signal_shutdown("Ark Configured")

def configure_ark(yaml_file):

    # Read the yammygang file
    yaml_lib = None
    with open(yaml_file, 'r') as stream:
        try:
            yaml_lib = yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            yaml_lib = None

    if yaml_lib:
        global started
        started = False
        print "Starting..."

        rospy.init_node("robot_ark_configurator")

        rospy.Subscriber("/ark_bridge/ark_config_settings_response", ArkConfigSettingsResponse, response_callback)
        time.sleep(0.3)
        started = True

        print "Configuring ARK"

        pub = rospy.Publisher("/ark_bridge/ark_config_settings_call", ArkConfigSettingsCall, latch=True, queue_size=1)
        settingsMsg = ArkConfigSettingsCall()

        settingsMsg.max_fwd_linear_speed = yaml_lib["ark_config"]["max_fwd_linear_speed"]
        settingsMsg.max_rev_linear_speed = yaml_lib["ark_config"]["max_rev_linear_speed"]
        settingsMsg.min_linear_speed = yaml_lib["ark_config"]["min_linear_speed"]
        settingsMsg.max_linear_acceleration = yaml_lib["ark_config"]["max_linear_acceleration"]
        settingsMsg.max_linear_deceleration = yaml_lib["ark_config"]["max_linear_deceleration"]
        settingsMsg.max_ang_speed = yaml_lib["ark_config"]["max_ang_speed"]
        settingsMsg.max_ang_accel = yaml_lib["ark_config"]["max_ang_accel"]
        settingsMsg.max_lateral_accel = yaml_lib["ark_config"]["max_lateral_accel"]
        settingsMsg.vehicle_length = yaml_lib["ark_config"]["vehicle_length"]
        settingsMsg.vehicle_width = yaml_lib["ark_config"]["vehicle_width"]
        settingsMsg.stopping_distance_1M = yaml_lib["ark_config"]["stopping_distance_1M"]
        settingsMsg.lidar_spacing = yaml_lib["ark_config"]["lidar_spacing"]
        settingsMsg.laser_fov = yaml_lib["ark_config"]["laser_fov"]
        settingsMsg.goal_tolerance_xy = yaml_lib["ark_config"]["goal_tolerance_xy"]
        settingsMsg.goal_tolerance_yaw = yaml_lib["ark_config"]["goal_tolerance_yaw"]
        settingsMsg.camera_3d_memory = yaml_lib["ark_config"]["camera_3d_memory"]
        settingsMsg.drive_direction = yaml_lib["ark_config"]["drive_direction"]

        pub.publish(settingsMsg)
        rospy.spin()

        print "Done"

if __name__ == "__main__":
    yaml_file_path = None
    if len(sys.argv) == 2:
      configure_ark(sys.argv[1])
    else:
      print "Not enough arguments!"
      print "Usage: configure_ark.py <filename.yaml>"
