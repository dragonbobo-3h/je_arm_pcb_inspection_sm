#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "je_arm_pcb_inspection_sm/behaviors/cb_move_mode_known_state.hpp"
#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"

namespace je_arm_pcb_inspection_sm
{

struct StPause;

namespace work_substates
{

struct StInspect;

namespace inspect_substates
{

struct StInspectWaitPlaceResult;

struct StInspectReturnWaitPose : smacc2::SmaccState<StInspectReturnWaitPose, StInspect>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<
      smacc2::EvCbSuccess<CbMoveModeKnownState, OrBothArms>,
      StInspectWaitPlaceResult>,
    smacc2::Transition<
      smacc2::EvCbFailure<CbMoveModeKnownState, OrBothArms>,
      StPause>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrBothArms, CbMoveModeKnownState>(
      "je_arm_pcb_inspection_sm",
      "dual_pick_p2.yaml");
  }

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kInspectResumeSubstateId),
      std::string(sm_data::kInspectSubstateReturnWaitPose));
    this->setGlobalSMData(std::string(sm_data::kResumeStateId), std::string(sm_data::kWorkState));
    this->setGlobalSMData(
      std::string(sm_data::kWorkResumeSubstateId),
      std::string(sm_data::kWorkSubstateInspect));
    RCLCPP_INFO(
      getLogger(),
      "WORK::INSPECT::RETURN_WAIT_POSE - moving both arms back to dual_pick_p2 before waiting for place result");
  }
};

}  // namespace inspect_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm