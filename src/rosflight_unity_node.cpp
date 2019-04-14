/**
 * @file rosflight_unity_node.cpp
 * @brief Entry point for ROSflight Unity SIL Simulation
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 13 April 2019
 */

#include <iostream>
#include <memory>

#include <ros/ros.h>

#include <rosflight.h>

#include "rosflight_unity/unity_board.h"
#include "rosflight_unity/unity_bridge.h"

std::unique_ptr<rosflight_unity::UnityBoard> board;
std::unique_ptr<rosflight_firmware::Mavlink> mavlink;
std::unique_ptr<rosflight_firmware::ROSflight> firmware;

// ----------------------------------------------------------------------------

void FixedUpdate(int32_t secs, int32_t nsecs)
{
  // Use Unity clock as the external clock
  board->setTime(secs, nsecs);

  firmware->run();
}

// ----------------------------------------------------------------------------

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rosflight_unity");

  rosflight_unity::UnityBridge unity;
  unity.init();

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
