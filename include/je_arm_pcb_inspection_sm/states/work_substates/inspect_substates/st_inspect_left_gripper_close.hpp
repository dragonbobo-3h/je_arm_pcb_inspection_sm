#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cl_moveit2z/client_behaviors/cb_ctrl_gripper.hpp>
#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/gripper_command_loader.hpp"

namespace je_arm_pcb_inspection_sm
{

struct StPause;

namespace work_substates
{

struct StInspect;

namespace inspect_substates
{

struct StInspectRightGripperOpen;

struct StInspectLeftGripperClose : smacc2::SmaccState<StInspectLeftGripperClose, StInspect>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<smacc2::EvCbSuccess<cl_moveit2z::CbCtrlGripper, OrGripper>, StInspectRightGripperOpen>,
    smacc2::Transition<smacc2::EvCbFailure<cl_moveit2z::CbCtrlGripper, OrGripper>, StPause>,
    smacc2::Transition<EvGripperClosed, StInspectRightGripperOpen>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  static void staticConfigure()
  {
    const auto cfg = je_arm_pcb_inspection_sm::utils::loadGripperCommandConfig("inspect_left_close");
    configure_orthogonal<OrGripper, cl_moveit2z::CbCtrlGripper>(
      cfg.mode, cfg.position, cfg.preset, cfg.leftValid, cfg.rightValid, cfg.topic, 1.5,
      "/joint_states_double_arm", 0.03, cfg.command, cfg.torque, cfg.waitForFeedback);
  }

  void onEntry()
  {
    this->setGlobalSMData(std::string(sm_data::kInspectResumeSubstateId), std::string(sm_data::kInspectSubstateLeftGripperClose));
    this->setGlobalSMData(std::string(sm_data::kWorkResumeSubstateId), std::string(sm_data::kWorkSubstateInspect));
    RCLCPP_INFO(getLogger(), "WORK::INSPECT::LEFT_GRIPPER_CLOSE - waiting left grip feedback [n bypass]");
  }
};

}  // namespace inspect_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm