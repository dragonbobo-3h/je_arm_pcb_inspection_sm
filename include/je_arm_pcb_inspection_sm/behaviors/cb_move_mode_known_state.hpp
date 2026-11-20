#pragma once

#include <filesystem>
#include <fstream>
#include <map>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cl_moveit2z/client_behaviors/cb_move_joints.hpp>
#include <yaml-cpp/yaml.h>

namespace je_arm_pcb_inspection_sm
{

/**
 * Joint-space known-state behavior that resolves the YAML directory from the
 * node's `joint_states_dir` parameter so `single` (joint1..joint7) and `dual`
 * (joint11..17 / joint21..27) arm modes can switch without overwriting configs.
 */
class CbMoveModeKnownState : public cl_moveit2z::CbMoveJoints
{
public:
  CbMoveModeKnownState(std::string pkg, std::string filename)
  : pkg_(std::move(pkg)), filename_(std::move(filename))
  {
  }

  void onEntry() override
  {
    std::string jointStatesDir = "config/move_group_client/joint_states";
    if (auto node = this->getNode())
    {
      node->get_parameter_or("joint_states_dir", jointStatesDir, jointStatesDir);
    }

    const std::string relativePath =
      jointStatesDir.empty() ? filename_ : (jointStatesDir + "/" + filename_);
    jointValueTarget_ = loadJointStatesFromFile(pkg_, relativePath);
    if (jointValueTarget_.empty())
    {
      RCLCPP_ERROR(
        getLogger(),
        "[CbMoveModeKnownState] Failed to load joint targets from %s/%s",
        pkg_.c_str(),
        relativePath.c_str());
      this->postFailureEvent();
      return;
    }

    cl_moveit2z::CbMoveJoints::onEntry();
  }

private:
  std::map<std::string, double> loadJointStatesFromFile(
    const std::string & pkg, const std::string & relativePath)
  {
    std::map<std::string, double> jointStates;
    std::string pkgPath;
    try
    {
      pkgPath = ament_index_cpp::get_package_share_directory(pkg);
    }
    catch (const std::exception & ex)
    {
      RCLCPP_ERROR(
        getLogger(),
        "[CbMoveModeKnownState] Package '%s' not found: %s",
        pkg.c_str(),
        ex.what());
      return jointStates;
    }

    const std::string filepath = pkgPath + "/" + relativePath;
    if (!std::filesystem::exists(filepath))
    {
      RCLCPP_ERROR(
        getLogger(),
        "[CbMoveModeKnownState] Joint state file does not exist: %s",
        filepath.c_str());
      return jointStates;
    }

    try
    {
      const YAML::Node root = YAML::LoadFile(filepath);
      const YAML::Node jointsNode = root["joint_states"];
      if (!jointsNode || !jointsNode.IsMap())
      {
        RCLCPP_ERROR(
          getLogger(),
          "[CbMoveModeKnownState] Missing 'joint_states' map in %s",
          filepath.c_str());
        return jointStates;
      }

      for (const auto & item : jointsNode)
      {
        jointStates[item.first.as<std::string>()] = item.second.as<double>();
      }

      if (root["scaling_factor"])
      {
        scalingFactor_ = root["scaling_factor"].as<double>();
      }

      RCLCPP_INFO(
        getLogger(),
        "[CbMoveModeKnownState] Loaded %zu joints from %s",
        jointStates.size(),
        filepath.c_str());
    }
    catch (const std::exception & ex)
    {
      RCLCPP_ERROR(
        getLogger(),
        "[CbMoveModeKnownState] Error parsing %s: %s",
        filepath.c_str(),
        ex.what());
      jointStates.clear();
    }

    return jointStates;
  }

  std::string pkg_;
  std::string filename_;
};

}  // namespace je_arm_pcb_inspection_sm
