// Copyright 2021 MyName/MyCompany Inc.
// Copyright 2021 RobosoftAI Inc. (template)
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

#include <memory>

#include <moveit_msgs/msg/planning_scene.hpp>

#include "rclcpp/rclcpp.hpp"
#include "smacc2/smacc.hpp"

// ORTHOGONALS
#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"
#include "je_arm_pcb_inspection_sm/orthogonals/or_keyboard.hpp"

#include <cl_moveit2z/cl_moveit2z.hpp>
#include <cl_moveit2z/client_behaviors.hpp>
#include <cl_keyboard/cl_keyboard.hpp>
#include <cl_keyboard/client_behaviors/cb_default_keyboard_behavior.hpp>
#include <smacc2/client_behaviors/cb_wait_topic_message.hpp>
#include <smacc2/smacc_updatable.hpp>

// EVENTS (must be after smacc2/smacc.hpp)
#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/logging.hpp"
#include "je_arm_pcb_inspection_sm/utils/planning_scene_obstacles.hpp"

using namespace cl_moveit2z;
using namespace cl_keyboard;
using namespace smacc2::default_events;

namespace je_arm_pcb_inspection_sm
{

// STATES - Forward declarations
struct StIdle;

//====================================
// STATE_MACHINE
//====================================

struct SmJeArmPcbInspection : public smacc2::SmaccStateMachineBase<SmJeArmPcbInspection, StIdle>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  je_arm_pcb_inspection_sm::utils::PlanningScenePublisher::SharedPtr planningScenePublisher_;

  void onInitialize() override 
  { 
    auto node = this->getNode();

    if (!node->has_parameter("publish_static_obstacles"))
    {
      node->declare_parameter<bool>("publish_static_obstacles", true);
    }
    if (!node->has_parameter("enable_gripper_control"))
    {
      node->declare_parameter<bool>("enable_gripper_control", false);
    }
    if (!node->has_parameter("simulated_gripper_action_sec"))
    {
      node->declare_parameter<double>("simulated_gripper_action_sec", 2.0);
    }
    if (!node->has_parameter("gripper_action_delay_sec"))
    {
      node->declare_parameter<double>("gripper_action_delay_sec", 0.0);
    }

    bool publishStaticObstacles = true;
    bool enableGripperControl = false;
    double simulatedGripperActionSec = 2.0;
    double gripperActionDelaySec = 0.0;

    node->get_parameter_or("publish_static_obstacles", publishStaticObstacles, true);
    node->get_parameter_or("enable_gripper_control", enableGripperControl, false);
    node->get_parameter_or("simulated_gripper_action_sec", simulatedGripperActionSec, 2.0);
    node->get_parameter_or("gripper_action_delay_sec", gripperActionDelaySec, 0.0);

    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] SM params | publish_static_obstacles=%d enable_gripper_control=%d simulated_gripper_action_sec=%.2f gripper_action_delay_sec=%.2f",
      log_utils::bjtNowString().c_str(),
      publishStaticObstacles,
      enableGripperControl,
      simulatedGripperActionSec,
      gripperActionDelaySec);

    this->setGlobalSMData(std::string(sm_data::kResumeStateId), std::string(sm_data::kWaitResourcesState));
    this->setGlobalSMData(std::string(sm_data::kResumeFromPause), false);
    this->setGlobalSMData(std::string(sm_data::kActivateResumeFromPause), false);
    this->setGlobalSMData(
      std::string(sm_data::kActivateResumeSubstateId),
      std::string(sm_data::kActivateSubstateP1));
      this->setGlobalSMData(std::string(sm_data::kBackToIdleResumeFromPause), false);
      this->setGlobalSMData(
        std::string(sm_data::kActivateDelayNextSubstateId),
        std::string(sm_data::kActivateSubstateP1));
    this->setGlobalSMData(std::string(sm_data::kWorkResumeSubstateId), std::string(sm_data::kWorkSubstatePick));
    this->setGlobalSMData(std::string(sm_data::kWorkDelayNextSubstateId), std::string(sm_data::kWorkSubstatePick));
    this->setGlobalSMData(std::string(sm_data::kPickResumeFromPause), false);
    this->setGlobalSMData(std::string(sm_data::kPickResumeSubstateId), std::string(sm_data::kPickSubstateLPregrasp));
    this->setGlobalSMData(std::string(sm_data::kPickDelayNextSubstateId), std::string(sm_data::kPickSubstateGripperOpen));
    this->setGlobalSMData(std::string(sm_data::kPlaceResumeFromPause), false);
    this->setGlobalSMData(std::string(sm_data::kPlaceResumeSubstateId), std::string(sm_data::kPlaceSubstateLPregrasp));
    this->setGlobalSMData(std::string(sm_data::kPlaceDelayNextSubstateId), std::string(sm_data::kPlaceSubstateLPregrasp));
    this->setGlobalSMData(std::string(sm_data::kTestGripperCommand), std::string("pick_close"));
    this->setGlobalSMData(std::string(sm_data::kPcbPresent), false);
    this->setGlobalSMData(std::string(sm_data::kLeftSlotFree), false);
    this->setGlobalSMData(std::string(sm_data::kRightSlotFree), false);
    this->setGlobalSMData(std::string(sm_data::kPcbTargetFrameId), std::string("base_link"));
    this->setGlobalSMData(std::string(sm_data::kPcbTargetX), -0.21966061203147025);
    this->setGlobalSMData(std::string(sm_data::kPcbTargetY), -0.5379023504590841);
    this->setGlobalSMData(std::string(sm_data::kPcbTargetZ), 0.48856085197117777);
    this->setGlobalSMData(std::string(sm_data::kPcbTargetQx), 0.11276390219028114);
    this->setGlobalSMData(std::string(sm_data::kPcbTargetQy), 0.9929116822698711);
    this->setGlobalSMData(std::string(sm_data::kPcbTargetQz), 0.01442683026805081);
    this->setGlobalSMData(std::string(sm_data::kPcbTargetQw), -0.03467794894817312);
    this->setGlobalSMData(std::string(sm_data::kPlaceTargetFrameId), std::string("base_link"));
    this->setGlobalSMData(std::string(sm_data::kPlaceTargetX), -0.3182978287647691);
    this->setGlobalSMData(std::string(sm_data::kPlaceTargetY), -0.30670100572292247);
    this->setGlobalSMData(std::string(sm_data::kPlaceTargetZ), 0.4037989942810653);
    this->setGlobalSMData(std::string(sm_data::kPlaceTargetQx), 0.9979704276802437);
    this->setGlobalSMData(std::string(sm_data::kPlaceTargetQy), -0.004350423415540621);
    this->setGlobalSMData(std::string(sm_data::kPlaceTargetQz), -0.06353018502134283);
    this->setGlobalSMData(std::string(sm_data::kPlaceTargetQw), 0.00012199578306026244);
    this->setGlobalSMData(std::string(sm_data::kWaitResourcesPoseFrameId), std::string("base_link"));
    this->setGlobalSMData(std::string(sm_data::kWaitResourcesPoseX), -0.2672000000000001);
    this->setGlobalSMData(std::string(sm_data::kWaitResourcesPoseY), -0.19259999999999997);
    this->setGlobalSMData(std::string(sm_data::kWaitResourcesPoseZ), 0.5103000000000001);
    this->setGlobalSMData(std::string(sm_data::kWaitResourcesPoseQx), 0.9979704276802437);
    this->setGlobalSMData(std::string(sm_data::kWaitResourcesPoseQy), -0.004350423415540621);
    this->setGlobalSMData(std::string(sm_data::kWaitResourcesPoseQz), -0.06353018502134283);
    this->setGlobalSMData(std::string(sm_data::kWaitResourcesPoseQw), 0.00012199578306026244);
    this->setGlobalSMData(std::string(sm_data::kWaitTimeoutSec), 120.0);
    this->setGlobalSMData(std::string(sm_data::kWorkCycleSec), 2.0);
    this->setGlobalSMData(std::string(sm_data::kBackHomeSec), 1.5);
    this->setGlobalSMData(std::string(sm_data::kSharedDelaySec), 0.6);

    this->setTransitionCallback(
      [this](const smacc2::introspection::SmaccTransitionInfo & transitionInfo)
      {
        const std::string source =
          transitionInfo.sourceState ? transitionInfo.sourceState->toShortName() : "<unknown>";
        const std::string target =
          transitionInfo.destinyState ? transitionInfo.destinyState->toShortName() : "<unknown>";
        const std::string event =
          transitionInfo.eventInfo ? transitionInfo.eventInfo->getEventTypeName() : "<unknown>";

        RCLCPP_INFO(
          log_utils::bizLogger(),
          "[%s] TRANSITION %s --(%s)--> %s",
          log_utils::bjtNowString().c_str(),
          source.c_str(),
          event.c_str(),
          target.c_str());
      });

    this->createOrthogonal<OrLeftArm>();
    this->createOrthogonal<OrRightArm>();
    this->createOrthogonal<OrBothArms>();
    this->createOrthogonal<OrGripper>();
    this->createOrthogonal<OrKeyboard>();

    if (publishStaticObstacles)
    {
      if (!je_arm_pcb_inspection_sm::utils::publishPlanningSceneObstacles(
          node, planningScenePublisher_))
      {
        RCLCPP_WARN(
          node->get_logger(),
          "State machine started without publishing static planning scene obstacles");
      }
    }
    else
    {
      RCLCPP_INFO(node->get_logger(), "Static obstacle publishing disabled by parameter");
    }
  }
};

}  // namespace je_arm_pcb_inspection_sm

// ====================================
// STATES - MUST be included AFTER SM definition
// ====================================

#include "states/st_idle.hpp"
#include "states/st_test_gripper.hpp"
#include "states/st_activate.hpp"
#include "states/st_back_to_idle.hpp"
#include "states/st_wait_resources.hpp"
#include "states/st_work.hpp"
#include "states/st_back.hpp"
#include "states/st_delay.hpp"
#include "states/st_pause_resume_router.hpp"
#include "states/st_pause.hpp"
