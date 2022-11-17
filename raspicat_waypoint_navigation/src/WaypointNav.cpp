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

#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2/utils.h"

#include "raspicat_waypoint_navigation/WaypointNav.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

using namespace ::std;

namespace waypoint_nav
{
WaypointNav::WaypointNav(ros::NodeHandle &nodeHandle, ros::NodeHandle &private_nodeHandle,
                         tf2_ros::Buffer &tf, bool waypoint_view_only)
    : nh_(nodeHandle),
      pnh_(private_nodeHandle),
      tf_(tf),
      ac_move_base_("move_base", true),
      waypoint_server_loader_("raspicat_waypoint_navigation",
                              "raspicat_navigation::BaseWaypointServer"),
      waypoint_rviz_loader_("raspicat_waypoint_navigation",
                            "raspicat_navigation::BaseWaypointRviz"),
      waypoint_nav_helper_loader_("raspicat_waypoint_navigation",
                                  "raspicat_navigation::WaypointNavHelperPlugin"),
      waypoint_radius_(3.0),
      waypoint_nav_start_(false)
{
  ros::AsyncSpinner spinner(pnh_.param("num_callback_threads", 4));
  spinner.start();

  if (not waypoint_view_only)
  {
    readParam();
    initPub();
    initActionClient();
    initSub();
    initClassLoader();
    getRobotPoseTimer();
    initServiceClient();
    registerDynamicParam();
    Run();
  }
  else
  {
    initPub();
    initClassLoader();
  }
}

WaypointNav::~WaypointNav() {}

void WaypointNav::readParam() {}

void WaypointNav::getRobotPoseTimer()
{
  sleep(1.0);
  get_robot_pose_timer_ = nh_.createTimer(
      ros::Duration(0.1), [&](auto &) { way_srv_->getRobotPose(tf_, WaypointNavStatus_); });
}

void WaypointNav::resolve_tf_between_map_and_robot_link()
{
  sleep(1.0);
  resolve_tf_timer_ = nh_.createTimer(ros::Duration(1.0), [&](auto &) {
    geometry_msgs::PoseWithCovarianceStamped msg;
    pnh_.getParam("/WaypointNav_node/initial_pose_x", WaypointNavStatus_.initial_pose_x);
    pnh_.getParam("/WaypointNav_node/initial_pose_y", WaypointNavStatus_.initial_pose_y);
    pnh_.getParam("/WaypointNav_node/initial_pose_a", WaypointNavStatus_.initial_pose_a);

    tf2::Quaternion q;
    q.setRPY(0, 0, static_cast<double>(WaypointNavStatus_.initial_pose_a));
    msg.pose.pose.position.x = WaypointNavStatus_.initial_pose_x;
    msg.pose.pose.position.y = WaypointNavStatus_.initial_pose_y;
    msg.pose.pose.orientation.z = q.getZ();
    msg.pose.pose.orientation.w = q.getW();
    msg.pose.covariance.at(0) = 0.25;
    msg.pose.covariance.at(7) = 0.25;
    msg.pose.covariance.at(35) = 0.06853892326654787;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    mcl_init_pose_.publish(msg);
  });
}

void WaypointNav::initPub()
{
  mcl_init_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);

  way_pose_array_ = nh_.advertise<geometry_msgs::PoseArray>("/waypoint", 1, true);
  way_area_array_ = nh_.advertise<visualization_msgs::MarkerArray>("/waypoint_area", 1, true);
  way_number_txt_array_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/waypoint_number_txt", 1, true);

  way_passed_ = nh_.advertise<std_msgs::Empty>("/waypoint_passed", 1, true);
  way_stop_ = nh_.advertise<std_msgs::Empty>("/waypoint_stop_function", 1, true);
  way_slope_ = nh_.advertise<std_msgs::Empty>("/waypoint_slope_function", 1, true);
  way_goal_ = nh_.advertise<std_msgs::Empty>("/waypoint_goal_function", 1, true);
  way_loop_ = nh_.advertise<std_msgs::Empty>("/waypoint_loop_function", 1, true);
  way_attention_speak_ =
      nh_.advertise<std_msgs::Empty>("/waypoint_attention_speak_function", 1, true);
  way_clear_costmap_ = nh_.advertise<std_msgs::Empty>("/waypoint_clear_costmap_function", 1, true);
}

void WaypointNav::initSub()
{
  sub_movebase_goal_ = nh_.subscribe("/move_base/status", 1, &WaypointNav::GoalReachedCb, this);
  sub_way_start_ = nh_.subscribe("/way_nav_start", 1, &WaypointNav::WaypointNavStartCb, this);
  sub_way_restart_ = nh_.subscribe("/way_nav_restart", 1, &WaypointNav::WaypointNavRestartCb, this);
}

void WaypointNav::initActionClient()
{
  ROS_INFO("Waiting for move_base Action Server to active.");
  resolve_tf_between_map_and_robot_link();
  sleep(3.0);
  ros::spinOnce();
  while (!ac_move_base_.waitForServer(ros::Duration(120.0)))
  {
    ROS_ERROR("move_base Action Server is not active.");
    exit(0);
  }
  resolve_tf_timer_.stop();
  ROS_INFO("move_base Action Server is active.");
}

void WaypointNav::initServiceClient()
{
  slope_obstacle_avoidance_client_["slope_obstacle_avoidance_on"] =
      nh_.serviceClient<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
          "slope_obstacle_avoidance_on");

  ROS_INFO("Waiting for SlopeObstacleAvoidance Server to active.");
  if (!slope_obstacle_avoidance_client_["slope_obstacle_avoidance_on"].waitForExistence(
          ros::Duration(100.0)))
  {
    ROS_ERROR("SlopeObstacleAvoidance Server is not active.");
    exit(0);
  }
  ROS_INFO("SlopeObstacleAvoidance Server is active.");

  slope_obstacle_avoidance_client_["slope_obstacle_avoidance_off"] =
      nh_.serviceClient<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
          "slope_obstacle_avoidance_off");

  ROS_INFO("Waiting for SlopeObstacleAvoidance Server to active.");
  if (!slope_obstacle_avoidance_client_["slope_obstacle_avoidance_off"].waitForExistence(
          ros::Duration(100.0)))
  {
    ROS_ERROR("SlopeObstacleAvoidance Server is not active.");
    exit(0);
  }
  ROS_INFO("SlopeObstacleAvoidance Server is active.");

  srv_way_nav_start_ = nh_.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
      "way_nav_start", [&](auto &req, auto &res) {
        ROS_INFO("Called service way_nav_start.");
        static bool call_once = false;
        if (not call_once)
        {
          call_once = true;
          waypoint_nav_start_ = true;
        }
        res.message = "Waypoint Navigation Start";
        res.success = true;
        return true;
      });

  waypoint_nav_start_restart_client_["way_nav_start"] =
      nh_.serviceClient<std_srvs::TriggerRequest, std_srvs::TriggerResponse>("way_nav_start");

  ROS_INFO("Waiting for WaypointNav Start Server to active.");
  if (!waypoint_nav_start_restart_client_["way_nav_start"].waitForExistence(ros::Duration(100.0)))
  {
    ROS_ERROR("WaypointNav Start Server is not active.");
    exit(0);
  }
  ROS_INFO("WaypointNav Start Server is active.");

  srv_way_nav_restart_ = nh_.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>(
      "way_nav_restart", [&](auto &req, auto &res) {
        ROS_INFO("Called service way_nav_restart.");
        if (waypoint_nav_start_)
        {
          timer_for_function_.erase("speak_stop");
          WaypointNavStatus_.flags.restart = true;
          res.message = "Waypoint Navigation Restart";
          res.success = true;
          return true;
        }
        else
        {
          static bool call_once = false;
          if (not call_once)
          {
            call_once = true;
            waypoint_nav_start_ = true;
          }
          res.message = "Waypoint Navigation Start";
          res.success = true;
          return true;
        }
      });

  waypoint_nav_start_restart_client_["way_nav_restart"] =
      nh_.serviceClient<std_srvs::TriggerRequest, std_srvs::TriggerResponse>("way_nav_restart");

  ROS_INFO("Waiting for WaypointNav Restart Server to active.");
  if (!waypoint_nav_start_restart_client_["way_nav_restart"].waitForExistence(ros::Duration(100.0)))
  {
    ROS_ERROR("WaypointNav Restart Server is not active.");
    exit(0);
  }
  ROS_INFO("WaypointNav Restart Server is active.");
}

void WaypointNav::initClassLoader()
{
  try
  {
    way_srv_ =
        waypoint_server_loader_.createInstance("raspicat_waypoint_navigation/WaypointServer");
    way_srv_->initialize(waypoint_server_);
    way_srv_->run();
    // way_srv_->checkWaypointYmal(pnh_);
    way_srv_->loadWaypointYmal(pnh_, waypoint_yaml_);
  }
  catch (pluginlib::PluginlibException &ex)
  {
    ROS_ERROR("failed to load add plugin. Error: %s", ex.what());
  }

  try
  {
    way_rviz_ = waypoint_rviz_loader_.createInstance("raspicat_waypoint_navigation/WaypointRviz");
    way_rviz_->initialize(waypoint_rviz_);
    way_rviz_->run();
    WaypointNavStatus_.waypoint_radius_threshold = waypoint_radius_;
    way_rviz_->WaypointRvizVisualization(waypoint_yaml_, way_pose_array_, way_area_array_,
                                         way_number_txt_array_,
                                         WaypointNavStatus_.waypoint_radius_threshold);
  }
  catch (pluginlib::PluginlibException &ex)
  {
    ROS_ERROR("failed to load add plugin. Error: %s", ex.what());
  }

  loadHelperPlugin("SlopeObstacleAvoidance", "raspicat_waypoint_navigation/SlopeObstacleAvoidance",
                   true);
  loadHelperPlugin("ClearCostMap", "raspicat_waypoint_navigation/ClearCostMap");
  loadHelperPlugin("ParamChange", "raspicat_waypoint_navigation/ParamChange");
  loadHelperPlugin("WaitingLine", "raspicat_waypoint_navigation/WaitingLine");
}

void WaypointNav::loadHelperPlugin(std::string class_name, std::string plugin_name, bool run)
{
  try
  {
    way_helper_[class_name] = waypoint_nav_helper_loader_.createInstance(plugin_name);
    way_helper_[class_name]->initialize(waypoint_nav_helper_);
    if (run) way_helper_[class_name]->run();
  }
  catch (pluginlib::PluginlibException &ex)
  {
    ROS_ERROR("failed to load add plugin. Error: %s", ex.what());
  }
}

void WaypointNav::registerDynamicParam()
{
  ddr_ = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(pnh_);
  ddr_->RegisterVariable((int *)&WaypointNavStatus_.waypoint_current_id, "waypoint_id");
  ddr_->PublishServicesTopics();
}

void WaypointNav::next_waypoint_function()
{
  if (WaypointNavStatus_.functions.next_waypoint.function)
    if (way_srv_->checkWaypointArea(waypoint_yaml_, WaypointNavStatus_, way_passed_))
      way_srv_->setNextWaypoint(ac_move_base_, goal_, waypoint_yaml_, WaypointNavStatus_);
}

void WaypointNav::stop_function()
{
  if (WaypointNavStatus_.functions.stop.function)
  {
    if (way_srv_->checkGoalReach(WaypointNavStatus_))
      if (WaypointNavStatus_.flags.restart)
      {
        timer_for_function_.erase("speak_stop");
        way_srv_->setNextWaypoint(ac_move_base_, goal_, waypoint_yaml_, WaypointNavStatus_);
        // way_srv_->setFalseWaypointFlag(WaypointNavStatus_);
      }
      else
      {
        // stop after param change function
        if (WaypointNavStatus_.flags.stop_after_and_keep &&
            not WaypointNavStatus_.flags.high_priority_proc)
        {
          if (not WaypointNavStatus_.flags.param_change)
          {
            for (auto i = 0; i < WaypointNavStatus_.functions.param_change.param_name.size(); i++)
              way_helper_["ParamChange"]->run(
                  WaypointNavStatus_.functions.param_change.param_name[i],
                  WaypointNavStatus_.functions.param_change.param_value[i]);

            WaypointNavStatus_.flags.param_change = true;
          }
        }

        if (timer_for_function_.find("speak_stop") == timer_for_function_.end())
        {
          ros::Timer speak_stop;
          speak_stop = nh_.createTimer(ros::Duration(0.1), [&](auto &) {
            std_msgs::Empty msg;
            way_stop_.publish(msg);
            sleep(6.0);
          });
          timer_for_function_["speak_stop"] = speak_stop;
        }
      }
  }
}

void WaypointNav::goal_function()
{
  if (WaypointNavStatus_.functions.goal.function)
    if (way_srv_->checkGoalReach(WaypointNavStatus_))
    {
      ROS_INFO("Waypoint Navigation Finish!");
      if (timer_for_function_.find("speak_goal") == timer_for_function_.end())
      {
        ros::Timer speak_goal;
        speak_goal = nh_.createTimer(ros::Duration(0.1), [&](auto &) {
          std_msgs::Empty msg;
          way_goal_.publish(msg);
          sleep(5.0);
        });
        timer_for_function_["speak_goal"] = speak_goal;
      }
    }
}

void WaypointNav::loop_function()
{
  if (WaypointNavStatus_.functions.loop.function)
    if (way_srv_->checkGoalReach(WaypointNavStatus_))
    {
      sleep(5.0);
      std_msgs::Empty msg;
      way_loop_.publish(msg);
      ROS_INFO("Waypoint Navigation Loop!");
      WaypointNavStatus_.waypoint_current_id = 0;
      way_srv_->setWaypoint(ac_move_base_, goal_, waypoint_yaml_, WaypointNavStatus_);
      timer_for_function_.erase(timer_for_function_.begin(), timer_for_function_.end());
    }
}

void WaypointNav::attention_speak_function_function()
{
  if (WaypointNavStatus_.functions.attention_speak.function)
    if (timer_for_function_.find("speak_attention") == timer_for_function_.end())
    {
      ros::Timer speak_attention;
      speak_attention = nh_.createTimer(ros::Duration(0.1), [&](auto &) {
        std_msgs::Empty msg;
        way_attention_speak_.publish(msg);
        sleep(5.0);
      });
      timer_for_function_["speak_attention"] = speak_attention;
    }
}

void WaypointNav::param_change_function_function()
{
  if (WaypointNavStatus_.functions.param_change.function &&
      not WaypointNavStatus_.flags.stop_after_and_keep)
  {
    if (not WaypointNavStatus_.flags.param_change)
    {
      for (auto i = 0; i < WaypointNavStatus_.functions.param_change.param_name.size(); i++)
        way_helper_["ParamChange"]->run(WaypointNavStatus_.functions.param_change.param_name[i],
                                        WaypointNavStatus_.functions.param_change.param_value[i]);

      WaypointNavStatus_.flags.param_change = true;
    }
  }
}

void WaypointNav::variable_waypoint_radius_function()
{
  if (not WaypointNavStatus_.functions.variable_waypoint_radius.function)
    WaypointNavStatus_.waypoint_radius_threshold = waypoint_radius_;
}

void WaypointNav::slope_function()
{
  if (WaypointNavStatus_.functions.slope.function)
  {
    static bool once_flag = false;

    if (!WaypointNavStatus_.flags.slope)
    {
      std_srvs::TriggerRequest req;
      std_srvs::TriggerResponse resp;
      if (!slope_obstacle_avoidance_client_["slope_obstacle_avoidance_on"].call(req, resp))
      {
        ROS_ERROR("Failed to invoke slope_obstacle_avoidance_on services.");
        exit(0);
      }

      WaypointNavStatus_.flags.slope = true;
      once_flag = false;
    }

    if (WaypointNavStatus_.flags.slope)
    {
      if (WaypointNavStatus_.slope_circle_area != 0 && not once_flag)
      {
        if (way_srv_->checkDistance(waypoint_yaml_, WaypointNavStatus_, "Circle"))
        {
          std_srvs::TriggerRequest req;
          std_srvs::TriggerResponse resp;
          if (!slope_obstacle_avoidance_client_["slope_obstacle_avoidance_off"].call(req, resp))
          {
            ROS_ERROR("Failed to invoke slope_obstacle_avoidance_off services.");
            exit(0);
          }
          once_flag = true;
        }
      }
    }
  }
}

void WaypointNav::clear_costmap_function()
{
  if ((WaypointNavStatus_.waypoint_previous_id != WaypointNavStatus_.waypoint_current_id) &&
      WaypointNavStatus_.functions.clear_costmap.function)
  {
    ac_move_base_.cancelAllGoals();
    sleep(1);
    way_helper_["ClearCostMap"]->run();
    sleep(5);
    std_msgs::Empty msg;
    way_clear_costmap_.publish(msg);
    sleep(5);
    way_srv_->sendWaypoint(ac_move_base_, goal_);
  }
}

void WaypointNav::waiting_line_function()
{
  if (WaypointNavStatus_.functions.waiting_line.function)
  {
    if (not WaypointNavStatus_.flags.waiting_line)
    {
      ac_move_base_.cancelAllGoals();

      way_helper_["WaitingLine"]->run();
      way_helper_["ParamChange"]->run("/move_base/global_costmap/obstacles_layer/enabled", "false");
      way_helper_["ParamChange"]->run("/move_base/local_costmap/obstacles_layer/enabled", "false");
      way_helper_["ParamChange"]->run(
          "/move_base/global_costmap/inflation_layer/cost_scaling_factor", "8");
      way_helper_["ParamChange"]->run("/move_base/DWAPlannerROS/max_vel_x", "0.3");
      way_helper_["ParamChange"]->run("/move_base/DWAPlannerROS/max_vel_trans", "0.3");

      sleep(20);

      way_srv_->sendWaypoint(ac_move_base_, goal_);
      WaypointNavStatus_.flags.waiting_line = true;
      // WaypointNavStatus_.flags.high_priority_proc = true;
    }

    // if (WaypointNavStatus_.flags.waiting_line && way_srv_->checkGoalReach(WaypointNavStatus_))
    // WaypointNavStatus_.flags.high_priority_proc = false;
  }
  else
    way_helper_["WaitingLine"]->finish();
}

void WaypointNav::clearSaveParam()
{
  if (not WaypointNavStatus_.servers.param_change.param_name_save.empty())
  {
    if (not WaypointNavStatus_.flags.stop_after_and_keep)
    {
      for (auto i = 0; i < WaypointNavStatus_.servers.param_change.param_name_save.size(); i++)
        way_helper_["ParamChange"]->run(
            WaypointNavStatus_.servers.param_change.param_name_save[i],
            WaypointNavStatus_.servers.param_change.param_value_save[i]);

      way_srv_->setWaypointFunction(waypoint_yaml_, WaypointNavStatus_);
      if (not WaypointNavStatus_.functions.param_change.function)
        way_srv_->clearSaveParam(WaypointNavStatus_);
    }
  }
}

void WaypointNav::obstacle_layer_controlle_function()
{
  if (WaypointNavStatus_.functions.obstacle_layer_controlle.function &&
      not WaypointNavStatus_.flags.obstacle_layer_controlle)
  {
    ac_move_base_.cancelAllGoals();

    if (WaypointNavStatus_.functions.obstacle_layer_controlle.enable)
    {
      way_helper_["ParamChange"]->run("/move_base/global_costmap/obstacles_layer/enabled", "true");
      way_helper_["ParamChange"]->run("/move_base/local_costmap/obstacles_layer/enabled", "true");
      way_helper_["ParamChange"]->run(
          "/move_base/global_costmap/inflation_layer/cost_scaling_factor", "1");
    }
    else
    {
      way_helper_["ParamChange"]->run("/move_base/global_costmap/obstacles_layer/enabled", "false");
      way_helper_["ParamChange"]->run("/move_base/local_costmap/obstacles_layer/enabled", "false");
    }
    sleep(40);

    way_srv_->sendWaypoint(ac_move_base_, goal_);
    WaypointNavStatus_.flags.obstacle_layer_controlle = true;
  }
}

void WaypointNav::stop_go_function()
{
  if (WaypointNavStatus_.functions.stop_go.function)
  {
    if (way_srv_->checkGoalReach(WaypointNavStatus_))
    {
      way_srv_->setNextWaypoint(ac_move_base_, goal_, waypoint_yaml_, WaypointNavStatus_);
    }
  }
}

void WaypointNav::Run()
{
  ROS_INFO("%s: Please ' rostopic pub -1 /way_nav_start std_msgs/Empty ' ",
           ros::this_node::getName().c_str());
  ROS_INFO("%s: Please ' rosservice call /way_nav_start ' ", ros::this_node::getName().c_str());

  ros::Rate loop_rate(5);
  while (ros::ok())
  {
    // Wait service call or topic pub
    if (waypoint_nav_start_)
    {
      static bool once_flag = false;
      if (not once_flag)
      {
        way_srv_->setWaypoint(ac_move_base_, goal_, waypoint_yaml_, WaypointNavStatus_);
        way_srv_->setFalseWaypointFunction(WaypointNavStatus_);
        once_flag = true;
      }

      WaypointNavStatus_.waypoint_previous_id = WaypointNavStatus_.waypoint_current_id;

      // Check the distance between the robot and the waypoint
      way_srv_->checkWaypointDistance(waypoint_yaml_, WaypointNavStatus_);

      // set function
      way_srv_->setWaypointFunction(waypoint_yaml_, WaypointNavStatus_);

      // function
      obstacle_layer_controlle_function();
      next_waypoint_function();
      waiting_line_function();
      stop_function();
      stop_go_function();
      goal_function();
      loop_function();
      attention_speak_function_function();
      param_change_function_function();
      variable_waypoint_radius_function();
      slope_function();
      clear_costmap_function();

      way_srv_->debug(WaypointNavStatus_);
      way_srv_->eraseTimer(WaypointNavStatus_, timer_for_function_);
      way_srv_->setFalseWaypointFunction(WaypointNavStatus_);

      // When change waypoint
      if (WaypointNavStatus_.waypoint_previous_id != WaypointNavStatus_.waypoint_current_id)
      {
        clearSaveParam();
        way_srv_->setFalseWaypointFlag(WaypointNavStatus_);
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void WaypointNav::WaypointNavStartCb(const std_msgs::EmptyConstPtr &msg)
{
  static bool once_flag = false;
  if (not once_flag)
  {
    once_flag = true;
    waypoint_nav_start_ = true;
  }
}

void WaypointNav::GoalReachedCb(const actionlib_msgs::GoalStatusArrayConstPtr &status)
{
  if (!status->status_list.empty())
  {
    actionlib_msgs::GoalStatus goalStatus = status->status_list[0];

    if (goalStatus.status == 3 && WaypointNavStatus_.flags.goal_reach == false)
      WaypointNavStatus_.flags.goal_reach = true;
  }
}

void WaypointNav::WaypointNavRestartCb(const std_msgs::EmptyConstPtr &msg)
{
  WaypointNavStatus_.flags.restart = true;
}

void WaypointNav::sleep(double time_s) { ros::Duration(time_s).sleep(); }

void WaypointNav::sleep(double &time_s) { ros::Duration(time_s).sleep(); }

}  // namespace waypoint_nav
