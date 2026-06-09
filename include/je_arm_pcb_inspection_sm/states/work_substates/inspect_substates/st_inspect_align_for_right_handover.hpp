#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/inspect_substates/cb_move_inspect_handover_joint_state.hpp"

namespace je_arm_pcb_inspection_sm
{

struct StPause;
struct StDelay;

namespace work_substates
{

struct StInspect;

namespace inspect_substates
{

struct StInspectRightGripperOpenReceive;

struct StInspectAlignForRightHandover : smacc2::SmaccState<StInspectAlignForRightHandover, StInspect>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<smacc2::EvCbSuccess<CbMoveInspectHandoverJointState, OrBothArms>, StDelay>,
    smacc2::Transition<smacc2::EvCbFailure<CbMoveInspectHandoverJointState, OrBothArms>, StPause>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrBothArms, CbMoveInspectHandoverJointState>("align_for_right_handover");
  }

  void onEntry()
  {
    this->setGlobalSMData(std::string(sm_data::kInspectResumeSubstateId), std::string(sm_data::kInspectSubstateAlignForRightHandover));
    this->setGlobalSMData(std::string(sm_data::kInspectDelayNextSubstateId), std::string(sm_data::kInspectSubstateRightBothGrippersClose));
    this->setGlobalSMData(std::string(sm_data::kResumeStateId), std::string(sm_data::kWorkState));
    this->setGlobalSMData(std::string(sm_data::kWorkResumeSubstateId), std::string(sm_data::kWorkSubstateInspect));
    this->setGlobalSMData(std::string(sm_data::kWorkDelayNextSubstateId), std::string(sm_data::kWorkSubstateInspect));
    RCLCPP_INFO(getLogger(), "WORK::INSPECT::ALIGN_TO_RIGHT - aligning both arms with a handover gap for right receive");
  }
};

}  // namespace inspect_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm