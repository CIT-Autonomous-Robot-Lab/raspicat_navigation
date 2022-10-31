#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "raspicat_waypoint_navigation/ParamChange.hpp"

namespace raspicat_navigation
{
void ParamChange::run(std::string string, std::string value)
{
  std::vector<std::string> split_slash_vec = splitSlash(string);
  std::string node = getNodeFromString(split_slash_vec);
  std::string param = getParamFromString(split_slash_vec);

  std::string param_change =
      "rosrun dynamic_reconfigure dynparam set " + node + " " + param + " " + value + " &";

  system(param_change.c_str());
}

std::vector<std::string> ParamChange::splitSlash(const std::string str)
{
  std::stringstream ss{str};
  std::string buf;
  std::vector<std::string> split_splash;

  while (std::getline(ss, buf, '/')) split_splash.push_back(buf);

  return split_splash;
}

std::string ParamChange::getNodeFromString(const std::vector<std::string> vec)
{
  std::vector<std::string> end_erase_vec = vec;
  end_erase_vec.pop_back();

  std::string node;
  for (auto str : end_erase_vec)
  {
    node += str;
    node += "/";
  }

  return node;
}

std::string ParamChange::getParamFromString(const std::vector<std::string> vec)
{
  return vec.back();
}
}  // namespace raspicat_navigation
PLUGINLIB_EXPORT_CLASS(raspicat_navigation::ParamChange,
                       raspicat_navigation::WaypointNavHelperPlugin)