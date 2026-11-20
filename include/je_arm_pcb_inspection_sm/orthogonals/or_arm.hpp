// Copyright 2021 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <chrono>
#include <string>

#include <cl_moveit2z/cl_moveit2z.hpp>
#include <cl_moveit2z/components/cp_motion_planner.hpp>
#include <cl_moveit2z/components/cp_trajectory_executor.hpp>
#include <cl_moveit2z/components/cp_trajectory_history.hpp>
#include <cl_moveit2z/components/cp_grasping_objects.hpp>

#include "cl_ros2_timer/cl_ros2_timer.hpp"
#include "smacc2/smacc.hpp"

namespace je_arm_pcb_inspection_sm
{
using namespace std::chrono_literals;

inline std::string getLeftTipLink(const rclcpp::Node::SharedPtr & node)
{
  std::string tipLink = "sdk_flange";
  if (node)
  {
    node->get_parameter_or("left_tip_link", tipLink, std::string("sdk_flange"));
  }
  return tipLink;
}

inline std::string getRightTipLink(const rclcpp::Node::SharedPtr & node)
{
  std::string tipLink = "Right_Link7";
  if (node)
  {
    node->get_parameter_or("right_tip_link", tipLink, std::string("Right_Link7"));
  }
  return tipLink;
}

class OrLeftArm : public smacc2::Orthogonal<OrLeftArm>
{
public:
  void onInitialize() override
  {
    auto node = this->getNode();
    bool enableVirtualGraspBoxes = false;
    std::string planningGroup = "jearm";
    node->get_parameter_or("enable_virtual_grasp_boxes", enableVirtualGraspBoxes, false);
    node->get_parameter_or("left_planning_group", planningGroup, std::string("jearm"));

    RCLCPP_INFO(
      node->get_logger(),
      "Initializing OrLeftArm with planning_group='%s', tip_link='%s'",
      planningGroup.c_str(),
      getLeftTipLink(node).c_str());

    auto move_group_client = this->createClient<cl_moveit2z::ClMoveit2z>(planningGroup);
    move_group_client->createComponent<cl_moveit2z::CpMotionPlanner>();
    move_group_client->createComponent<cl_moveit2z::CpTrajectoryExecutor>();
    move_group_client->createComponent<cl_moveit2z::CpTrajectoryHistory>();
    if (!enableVirtualGraspBoxes)
    {
      RCLCPP_INFO(node->get_logger(), "Skipping left arm virtual grasp box creation");
      return;
    }

    auto graspingComponent = move_group_client->createComponent<cl_moveit2z::CpGraspingComponent>();

    graspingComponent->gripperLink_ = getLeftTipLink(node);
    graspingComponent->createGraspableBox("virtualBox", 0, 0.5, 0.5, 0.1, 0.1, 0.1);
  }
};

class OrRightArm : public smacc2::Orthogonal<OrRightArm>
{
public:
  void onInitialize() override
  {
    auto node = this->getNode();
    bool enableVirtualGraspBoxes = false;
    std::string planningGroup = "right_arm";
    node->get_parameter_or("enable_virtual_grasp_boxes", enableVirtualGraspBoxes, false);
    node->get_parameter_or("right_planning_group", planningGroup, std::string("right_arm"));

    RCLCPP_INFO(
      node->get_logger(),
      "Initializing OrRightArm with planning_group='%s', tip_link='%s'",
      planningGroup.c_str(),
      getRightTipLink(node).c_str());

    auto move_group_client = this->createClient<cl_moveit2z::ClMoveit2z>(planningGroup);
    move_group_client->createComponent<cl_moveit2z::CpMotionPlanner>();
    move_group_client->createComponent<cl_moveit2z::CpTrajectoryExecutor>();
    move_group_client->createComponent<cl_moveit2z::CpTrajectoryHistory>();
    if (!enableVirtualGraspBoxes)
    {
      RCLCPP_INFO(node->get_logger(), "Skipping right arm virtual grasp box creation");
      return;
    }

    auto graspingComponent = move_group_client->createComponent<cl_moveit2z::CpGraspingComponent>();

    graspingComponent->gripperLink_ = getRightTipLink(node);
    graspingComponent->createGraspableBox("virtualBox", 0, 0.5, 0.5, 0.1, 0.1, 0.1);
  }
};

class OrBothArms : public smacc2::Orthogonal<OrBothArms>
{
public:
  void onInitialize() override
  {
    auto node = this->getNode();
    std::string planningGroup = "jearm";
    node->get_parameter_or("both_planning_group", planningGroup, std::string("jearm"));

    RCLCPP_INFO(
      node->get_logger(),
      "Initializing OrBothArms with planning_group='%s'",
      planningGroup.c_str());

    auto move_group_client = this->createClient<cl_moveit2z::ClMoveit2z>(planningGroup);
    move_group_client->createComponent<cl_moveit2z::CpMotionPlanner>();
    move_group_client->createComponent<cl_moveit2z::CpTrajectoryExecutor>();
    move_group_client->createComponent<cl_moveit2z::CpTrajectoryHistory>();
  }
};

class OrGripper : public smacc2::Orthogonal<OrGripper>
{
public:
  void onInitialize() override {}
};

using OrArm = OrLeftArm;
}  // namespace je_arm_pcb_inspection_sm
