#pragma once

#include <cl_moveit2z/client_behaviors/cb_move_known_state.hpp>
#include <smacc2/smacc.hpp>

#include "je_arm_pcb_inspection_sm/components/cp_top_level_flow.hpp"
#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/logging.hpp"

namespace je_arm_pcb_inspection_sm
{
struct StBack;
struct StDelay;
struct StPause;

namespace work_substates
{
struct StPlace;

namespace place_substates
{

// Carry the PCB via the existing BACK waypoint before approaching the pickup
// fixture. Then reuse PLACE's approach/down/open/up/close sequence, with the
// saved PICK pose and pick_offset instead of the live PLACE target.
struct StPlaceReturnToPick : smacc2::SmaccState<StPlaceReturnToPick, StPlace>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<smacc2::EvCbSuccess<cl_moveit2z::CbMoveKnownState, OrArm>, StDelay>,
    smacc2::Transition<smacc2::EvCbFailure<cl_moveit2z::CbMoveKnownState, OrArm>, StPause>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrArm, cl_moveit2z::CbMoveKnownState>(
      "je_arm_pcb_inspection_sm", "config/move_group_client/joint_states/dual_pick_p2.yaml");
  }

  void onEntry()
  {
    this->setGlobalSMData(std::string(sm_data::kReturningToPick), true);
    this->setGlobalSMData(
      std::string(sm_data::kPlaceResumeSubstateId),
      std::string(sm_data::kPlaceSubstateReturnToPick));
    this->setGlobalSMData(
      std::string(sm_data::kPlaceDelayNextSubstateId),
      std::string(sm_data::kPlaceSubstateLPregrasp));
    this->setGlobalSMData(
      std::string(sm_data::kResumeStateId), std::string(sm_data::kWorkState));
    this->setGlobalSMData(
      std::string(sm_data::kWorkResumeSubstateId), std::string(sm_data::kWorkSubstatePlace));
    this->setGlobalSMData(
      std::string(sm_data::kWorkDelayNextSubstateId), std::string(sm_data::kWorkSubstatePlace));
    RCLCPP_INFO(
      log_utils::bizLogger(),
      "LOOP RETURN_TO_PICK - keeping gripper closed, moving via pick_p2 to saved pickup pose");
  }
};

// Only behavior success reaches here; manual bypass/fault/PAUSE must not
// make a still-held PCB available for the next PICK.
struct StPlaceCycleComplete : smacc2::SmaccState<StPlaceCycleComplete, StPlace>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<smacc2::Transition<EvPlaceDone, StBack>> reactions;

  void onEntry()
  {
    bool loopMode = false;
    bool returningToPick = false;
    this->getGlobalSMData(std::string(sm_data::kLoopMode), loopMode);
    this->getGlobalSMData(std::string(sm_data::kReturningToPick), returningToPick);
    if (loopMode && returningToPick)
    {
      CpTopLevelFlow * flow = nullptr;
      this->requiresComponent(flow);
      flow->completeLoopCycle();
      RCLCPP_INFO(log_utils::bizLogger(), "LOOP PCB returned - BACK then next PICK");
    }
    this->template postEvent<EvPlaceDone>();
  }
};

}  // namespace place_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm