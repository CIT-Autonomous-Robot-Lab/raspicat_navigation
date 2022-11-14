/*
 *Copyright 2022, Tatsuhiro Ikebe.
 *
 *Licensed under the Apache License, Version 2.0 (the "License");
 *you may not use this file except in compliance with the License.
 *You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *Unless required by applicable law or agreed to in writing, software
 *distributed under the License is distributed on an "AS IS" BASIS,
 *WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *See the License for the specific language governing permissions and
 *limitations under the License.
 */

#ifndef WAITING_LINE_HPP_
#define WAITING_LINE_HPP_

#include <ros/ros.h>

#include "raspicat_waypoint_navigation/WaypointNavHelperPlugin.hpp"

namespace raspicat_navigation
{
class WaitingLine : public raspicat_navigation::WaypointNavHelperPlugin
{
  ros::NodeHandle nh_, pnh_;
  ros::Publisher stop_vel_publisher;
  ros::Subscriber scan_subscriber_;
  ros::Timer waiting_line_timer_;

  sensor_msgs::LaserScan scan_;

 public:
  void initialize(std::string name)
  {
    ROS_INFO("raspicat_navigation::WaitingLine initialize");
    initPubSub();
  }

  void run()
  {
    ROS_INFO("raspicat_navigation::WaitingLine run");
    waitingLine();
  }
  void run(std::string param, std::string value) {}
  void finish() { waiting_line_timer_.stop(); }

  void initPubSub();

  void waitingLine();
  bool checkObstacle(sensor_msgs::LaserScan scan);
  void stopVelPublish();

  double calcExtraAngle();
  bool checkBoxRangeSearch(double x, double y);

  bool get_scan_;
};

constexpr double degree90_radian = 1.5708;
}  // namespace raspicat_navigation
#endif  // WaitingLine