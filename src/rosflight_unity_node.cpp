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

class ROSflightUnity
{
public:
  ROSflightUnity(ros::NodeHandle& nh)
  : nh_(nh)
  {
    sub_rc_ = nh.subscribe("rc_in", 1, &ROSflightUnity::rc_callback, this);
    pub_truth_ = nh.advertise<geometry_msgs::PoseStamped>("truth", 1);

    //
    // Unity bridge setup
    //

    unity_.reset(new rosflight_unity::UnityBridge);
    unity_->init();

    unity_->doConfigSim();
    unity_->doConfigVehicle();

    //
    // Initialize ROSflight autopilot
    //

    board_.reset(new rosflight_unity::UnityBoard(*unity_));
    mavlink_.reset(new rosflight_firmware::Mavlink(*board_));
    firmware_.reset(new rosflight_firmware::ROSflight(*board_, *mavlink_));

    firmware_->init();

    // Register a listener to the Unity physics update event
    unity_->onPhysicsUpdate(std::bind(&ROSflightUnity::FixedUpdate, this,
                                std::placeholders::_1, std::placeholders::_2));
  }

  ~ROSflightUnity() = default;

private:
  // ros
  ros::NodeHandle nh_;
  ros::Subscriber sub_rc_;
  ros::Publisher pub_truth_;

  // ROSflight SIL objects
  std::unique_ptr<rosflight_unity::UnityBridge> unity_;
  std::unique_ptr<rosflight_unity::UnityBoard> board_;
  std::unique_ptr<rosflight_firmware::Mavlink> mavlink_;
  std::unique_ptr<rosflight_firmware::ROSflight> firmware_;

  void FixedUpdate(int32_t secs, int32_t nsecs)
  {
    //
    // ROS Communications
    //

    publishTruth();

    //
    // Software-in-the-loop
    //

    // Use Unity clock as the external clock
    board_->setTime(secs, nsecs);

    firmware_->run();
  }

  // --------------------------------------------------------------------------

  void rc_callback(const rosflight_msgs::RCRawConstPtr& msg)
  {
    board_->setRC(msg->values.data());
  }

  // --------------------------------------------------------------------------

  void publishTruth()
  {
    auto truth = unity_->getTruthData();
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time(truth.timestamp_secs, truth.timestamp_nsecs);
    msg.header.frame_id = "world_ned";
    msg.pose.position.x = truth.x[0];
    msg.pose.position.y = truth.x[1];
    msg.pose.position.z = truth.x[2];
    msg.pose.orientation.x = truth.q[1];
    msg.pose.orientation.y = truth.q[2];
    msg.pose.orientation.z = truth.q[3];
    msg.pose.orientation.w = truth.q[0];
    pub_truth_.publish(msg);
  }

};

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rosflight_unity");
  ros::NodeHandle nh("~");
  ROSflightUnity sil(nh);
  ros::spin();
  return 0;
}
