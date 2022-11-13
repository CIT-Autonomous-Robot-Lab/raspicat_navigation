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

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include "raspicat_waypoint_navigation/WaitingLine.hpp"

namespace raspicat_navigation
{
void WaitingLine::initPubSub()
{
  stop_vel_publisher = nh_.advertise<geometry_msgs::Twist>("/stop_vel", 1, true);

  scan_subscriber_ = pnh_.subscribe<sensor_msgs::LaserScan>("/scan", 1, [&](const auto &scan) {
    scan_ = *scan;
    get_scan_ = true;
  });
}

void WaitingLine::waitingLine()
{
  waiting_line_timer_ = nh_.createTimer(ros::Duration(0.1), [&](auto &) {
    if (get_scan_)
      if (checkObstacle()) stopVelPublish();
  });
}

bool WaitingLine::checkObstacle()
{
  double extra_angle = calcExtraAngle();
  uint16_t cnt_obstacle = 0;
  double angle = 0;
  for (auto &scan_data : scan_.ranges)
  {
    if (extra_angle < angle && angle < scan_.angle_max * 2 - extra_angle)
    {
      double x = fabs(scan_data * cos(fabs(angle - extra_angle)));
      double y = fabs(scan_data * sin(fabs(angle - extra_angle)));

      checkBoxRangeSearch(x, y) ? ++cnt_obstacle : false;
    }

    if (cnt_obstacle > 10) return true;

    angle += scan_.angle_increment;
  }

  return false;
}

double WaitingLine::calcExtraAngle() { return fabs(degree90_radian - scan_.angle_max); }

bool WaitingLine::checkBoxRangeSearch(double x, double y)
{
  return (x <= 0.15 && y <= 1.0) ? true : false;
}

void WaitingLine::stopVelPublish()
{
  static geometry_msgs::Twist stop_vel;
  stop_vel_publisher.publish(stop_vel);
}
}  // namespace raspicat_navigation
PLUGINLIB_EXPORT_CLASS(raspicat_navigation::WaitingLine,
                       raspicat_navigation::WaypointNavHelperPlugin)