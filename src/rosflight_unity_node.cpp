/**
 * @file rosflight_unity_node.cpp
 * @brief Entry point for ROSflight Unity SIL Simulation
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 13 April 2019
 */

#include <iostream>
#include <memory>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <rosflight.h>
#include <rosflight_msgs/RCRaw.h>

#include "rosflight_unity/unity_board.h"
#include "rosflight_unity/unity_bridge.h"

std::unique_ptr<rosflight_unity::UnityBoard> board;
std::unique_ptr<rosflight_firmware::Mavlink> mavlink;
std::unique_ptr<rosflight_firmware::ROSflight> firmware;

// ----------------------------------------------------------------------------

void FixedUpdate(int32_t secs, int32_t nsecs)
{

  //
  // ROS Communications
  //

  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();


  //
  // Software-in-the-loop
  //

  // Use Unity clock as the external clock
  board->setTime(secs, nsecs);

  firmware->run();
}

// ----------------------------------------------------------------------------

void rc_callback(const rosflight_msgs::RCRawConstPtr& msg)
{
  board->setRC(msg->values.data());
}

// ----------------------------------------------------------------------------

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rosflight_unity");

  //
  // ROS connections
  //

  ros::NodeHandle nh("~");

  ros::Subscriber sub_rc = nh.subscribe("rc_in", 1, rc_callback);
  // pub_truth = nh.advertise<>("rc_in", 1, rc_callback);

  //
  // Unity bridge setup
  //

  rosflight_unity::UnityBridge unity;
  unity.init();

  unity.doConfigSim();
  unity.doConfigVehicle();

  //
  // Initialize ROSflight autopilot
  //

  board.reset(new rosflight_unity::UnityBoard(unity));
  mavlink.reset(new rosflight_firmware::Mavlink(*board));
  firmware.reset(new rosflight_firmware::ROSflight(*board, *mavlink));

  firmware->init();

  // Register a listener to the Unity physics update event
  unity.onPhysicsUpdate(std::bind(FixedUpdate,
                          std::placeholders::_1, std::placeholders::_2));

  ros::spin();
  return 0;
}
