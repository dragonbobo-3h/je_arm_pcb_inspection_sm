#pragma once

#include <algorithm>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <je_software/msg/end_effector_command.hpp>
#include <yaml-cpp/yaml.h>

namespace je_arm_pcb_inspection_sm
{
namespace utils
{

struct GripperCommandConfig
{
  int mode{je_software::msg::EndEffectorCommand::MODE_POSITION};
  double position{0.0};
  int preset{0};
  std::string command;
  double torque{0.0};
  bool forcePositionMode{true};
  bool leftValid{true};
  bool rightValid{false};
  bool waitForFeedback{true};
  double delaySec{-1.0};
  std::string topic{"/end_effector_cmd_lr"};
  std::string sourcePath;
  bool loaded{false};
};

inline GripperCommandConfig defaultGripperCommandConfig(const std::string & commandName)
{
  GripperCommandConfig cfg;
  if (commandName.find("_open") != std::string::npos)
  {
    cfg.position = 1.0;
  }
  else
  {
    cfg.position = 0.0;
  }
  return cfg;
}

inline int parseGripperMode(const YAML::Node & node, int defaultMode)
{
  if (!node)
  {
    return defaultMode;
  }

  if (node.IsScalar())
  {
    const auto modeStringRaw = node.as<std::string>();
    std::string modeString = modeStringRaw;
    std::transform(modeString.begin(), modeString.end(), modeString.begin(), ::tolower);

    if (modeString == "position")
    {
      return je_software::msg::EndEffectorCommand::MODE_POSITION;
    }
    if (modeString == "preset")
    {
      return je_software::msg::EndEffectorCommand::MODE_PRESET;
    }
    if (modeString == "torque")
    {
      return je_software::msg::EndEffectorCommand::MODE_TORQUE;
    }

    return node.as<int>();
  }

  return defaultMode;
}

inline GripperCommandConfig loadGripperCommandConfig(const std::string & commandName)
{
  GripperCommandConfig cfg = defaultGripperCommandConfig(commandName);

  try
  {
    cfg.sourcePath =
      ament_index_cpp::get_package_share_directory("je_arm_pcb_inspection_sm") +
      "/config/gripper_commands.yaml";

    const YAML::Node root = YAML::LoadFile(cfg.sourcePath);
    const YAML::Node common = root["gripper_common"];
    const YAML::Node commands = root["gripper_commands"];
    const YAML::Node command = commands ? commands[commandName] : YAML::Node();

    if (common)
    {
      if (common["force_position_mode"])
      {
        cfg.forcePositionMode = common["force_position_mode"].as<bool>();
      }
      if (common["topic"])
      {
        cfg.topic = common["topic"].as<std::string>();
      }
      if (common["left_valid"])
      {
        cfg.leftValid = common["left_valid"].as<bool>();
      }
      if (common["right_valid"])
      {
        cfg.rightValid = common["right_valid"].as<bool>();
      }
      if (common["wait_for_feedback"])
      {
        cfg.waitForFeedback = common["wait_for_feedback"].as<bool>();
      }
      if (common["delay_sec"])
      {
        cfg.delaySec = common["delay_sec"].as<double>();
      }
      if (common["action_delay_sec"])
      {
        cfg.delaySec = common["action_delay_sec"].as<double>();
      }
    }

    if (command)
    {
      cfg.mode = parseGripperMode(command["mode"], cfg.mode);
      if (command["position"])
      {
        cfg.position = command["position"].as<double>();
      }
      if (command["preset"])
      {
        cfg.preset = command["preset"].as<int>();
      }
      if (command["command"])
      {
        cfg.command = command["command"].as<std::string>();
      }
      if (command["torque"])
      {
        cfg.torque = command["torque"].as<double>();
      }
      if (command["topic"])
      {
        cfg.topic = command["topic"].as<std::string>();
      }
      if (command["left_valid"])
      {
        cfg.leftValid = command["left_valid"].as<bool>();
      }
      if (command["right_valid"])
      {
        cfg.rightValid = command["right_valid"].as<bool>();
      }
      if (command["wait_for_feedback"])
      {
        cfg.waitForFeedback = command["wait_for_feedback"].as<bool>();
      }
      if (command["delay_sec"])
      {
        cfg.delaySec = command["delay_sec"].as<double>();
      }
      if (command["action_delay_sec"])
      {
        cfg.delaySec = command["action_delay_sec"].as<double>();
      }
    }

    if (cfg.forcePositionMode)
    {
      cfg.mode = je_software::msg::EndEffectorCommand::MODE_POSITION;
    }

    cfg.loaded = true;
  }
  catch (...)
  {
    cfg.loaded = false;
    cfg.sourcePath.clear();
  }

  return cfg;
}

}  // namespace utils
}  // namespace je_arm_pcb_inspection_sm
