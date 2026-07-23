#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/inspect_substates/cb_move_inspect_handover_pose.hpp"

namespace je_arm_pcb_inspection_sm
{

struct StPause;
struct StDelay;

namespace work_substates
{

struct StInspect;

namespace inspect_substates
{

struct StInspectLeftGripperClose;

struct StInspectLeftApproach : smacc2::SmaccState<StInspectLeftApproach, StInspect>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<smacc2::EvCbSuccess<CbMoveInspectHandoverPose, OrLeftArm>, StDelay>,
    smacc2::Transition<smacc2::EvCbFailure<CbMoveInspectHandoverPose, OrLeftArm>, StPause>,
    smacc2::Transition<EvInspectStepAck, StInspectLeftGripperClose>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrLeftArm, CbMoveInspectHandoverPose>("left_receive", "Link17");
  }

  void onEntry()
  {
    this->setGlobalSMData(std::string(sm_data::kInspectResumeSubstateId), std::string(sm_data::kInspectSubstateLeftApproach));
    this->setGlobalSMData(std::string(sm_data::kInspectDelayNextSubstateId), std::string(sm_data::kInspectSubstateLeftGripperClose));
    this->setGlobalSMData(std::string(sm_data::kResumeStateId), std::string(sm_data::kWorkState));
    this->setGlobalSMData(std::string(sm_data::kWorkResumeSubstateId), std::string(sm_data::kWorkSubstateInspect));
    this->setGlobalSMData(std::string(sm_data::kWorkDelayNextSubstateId), std::string(sm_data::kWorkSubstateInspect));
    RCLCPP_INFO(getLogger(), "WORK::INSPECT::LEFT_APPROACH - cartesian linear left arm handover approach [n bypass]");
  }
};

}  // namespace inspect_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm