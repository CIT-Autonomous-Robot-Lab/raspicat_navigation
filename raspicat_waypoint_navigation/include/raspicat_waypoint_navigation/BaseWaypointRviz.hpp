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

#ifndef BASE_WAYPOINT_RVIZ_HPP_
#define BASE_WAYPOINT_RVIZ_HPP_

#include <ros/ros.h>

using namespace ::std;

namespace raspicat_navigation
{
class BaseWaypointRviz
{
 public:
  virtual void initialize(std::string name) = 0;
  virtual void run() = 0;
  virtual void WaypointRvizVisualization(XmlRpc::XmlRpcValue &waypoint_yaml,
                                         ros::Publisher &way_pose_array_,
                                         ros::Publisher &way_area_array_,
                                         ros::Publisher &way_number_txt_array_,
                                         float &waypoint_area_threshold_) = 0;

  virtual ~BaseWaypointRviz() {}

 protected:
  BaseWaypointRviz() {}
};

}  // namespace raspicat_navigation
#endif  // BASE_WAYPOINT_RVIZ_HPP_