#pragma once

#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>

namespace je_arm_pcb_inspection_sm
{
namespace utils
{

inline bool loadWaitResourcesPose(
  geometry_msgs::msg::PoseStamped & pose, std::string & sourcePath, std::string & error)
{
  try
  {
    sourcePath =
      ament_index_cpp::get_package_share_directory("je_arm_pcb_inspection_sm") +
      "/config/move_group_client/cartesian_states/wait_resources.yaml";

    const YAML::Node root = YAML::LoadFile(sourcePath);
    pose.header.frame_id = root["frame_id"].as<std::string>();
    pose.pose.position.x = root["x"].as<double>();
    pose.pose.position.y = root["y"].as<double>();
    pose.pose.position.z = root["z"].as<double>();
    pose.pose.orientation.x = root["qx"].as<double>();
    pose.pose.orientation.y = root["qy"].as<double>();
    pose.pose.orientation.z = root["qz"].as<double>();
    pose.pose.orientation.w = root["qw"].as<double>();
    return true;
  }
  catch (const std::exception & e)
  {
    error = e.what();
    return false;
  }
}

}  // namespace utils
}  // namespace je_arm_pcb_inspection_sm
