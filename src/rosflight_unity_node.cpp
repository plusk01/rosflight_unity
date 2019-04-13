/**
 * @file rosflight_unity_node.cpp
 * @brief Entry point for ROSflight Unity SIL Simulation
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 13 April 2019
 */

#include <ros/ros.h>

#include "rosflight_unity/unity_board.h"
#include <rosflight.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rosflight_unity");

  rosflight_unity::UnityBoard board;
  rosflight_firmware::Mavlink mavlink(board);
  rosflight_firmware::ROSflight firmware(board, mavlink);

  firmware.init();

  while (true)
  {
    firmware.run();
  }

  return 0;
}
