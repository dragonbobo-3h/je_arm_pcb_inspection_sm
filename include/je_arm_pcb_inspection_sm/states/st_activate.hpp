#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "je_arm_pcb_inspection_sm/behaviors/cb_move_mode_known_state.hpp"
#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/logging.hpp"

namespace je_arm_pcb_inspection_sm
{

struct SmJeArmPcbInspection;
struct StWaitResources;
struct StDelay;
struct StPause;

// Forward declarations for Activate sub-states
struct StActivateResumeRouter;
struct StActivateP1;
struct StActivateP2;

// ---- Composite parent: StActivate ------------------------------------------
// Sequences P1 -> (StDelay) -> P2 -> (StDelay) -> StWaitResources.
//
// Pause safety: P1/P2 do NOT handle EvPauseRequested. Pressing 'p' during
// motion is silently ignored (event bubbles to SM root unhandled). Pause only
// takes effect when sitting in the intermediate StDelay between waypoints —
// exactly mirroring the pregrasp P1/P2/P3 pattern.
struct StActivate : smacc2::SmaccState<StActivate, SmJeArmPcbInspection, StActivateResumeRouter>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvPauseRequested, StPause>,
    smacc2::Transition<EvCanWork, StWaitResources>,
    smacc2::Transition<EvDelayRequested, StDelay>
  > reactions;

  void onEntry()
  {
    // Inform StDelay and StPauseResumeRouter that we are in the Activate phase.
    this->setGlobalSMData(
      std::string(sm_data::kResumeStateId),
      std::string(sm_data::kActivateState));
    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] ENTER ACTIVATE",
      log_utils::bjtNowString().c_str());
  }

  void onExit()
  {
    RCLCPP_DEBUG(
      log_utils::bizLogger(), "[%s] EXIT ACTIVATE", log_utils::bjtNowString().c_str());
  }
};

// ---- Resume router: dispatches to the correct Activate sub-state -----------
struct StActivateResumeRouter : smacc2::SmaccState<StActivateResumeRouter, StActivate>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvActivateResumeToP1, StActivateP1>,
    smacc2::Transition<EvActivateResumeToP2, StActivateP2>,
    smacc2::Transition<EvCanWork, StWaitResources>
  > reactions;

  void onEntry()
  {
    std::string substate = sm_data::kActivateSubstateP1;
    this->getGlobalSMData(std::string(sm_data::kActivateResumeSubstateId), substate);

    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] ACTIVATE::RESUME_ROUTER --> %s",
      log_utils::bjtNowString().c_str(),
      substate.c_str());

    if (substate == sm_data::kActivateSubstateP2)
    {
      this->template postEvent<EvActivateResumeToP2>();
    }
    else if (substate == sm_data::kActivateSubstateDone)
    {
      // P2 already completed — skip straight to WaitResources.
      this->template postEvent<EvCanWork>();
    }
    else
    {
      // P1 or any unknown value: start from the beginning.
      this->template postEvent<EvActivateResumeToP1>();
    }
  }
};

// ---- P1: first approach waypoint -------------------------------------------
struct StActivateP1 : smacc2::SmaccState<StActivateP1, StActivate>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<smacc2::EvCbSuccess<CbMoveModeKnownState, OrBothArms>, StDelay>,
    smacc2::Transition<smacc2::EvCbFailure<CbMoveModeKnownState, OrBothArms>, StPause>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrBothArms, CbMoveModeKnownState>(
      "je_arm_pcb_inspection_sm",
      "dual_pick_p1.yaml");
  }

  void onEntry()
  {
    // Mark current sub-state; delay will advance pointer to P2.
    this->setGlobalSMData(
      std::string(sm_data::kActivateResumeSubstateId),
      std::string(sm_data::kActivateSubstateP1));
    this->setGlobalSMData(
      std::string(sm_data::kActivateDelayNextSubstateId),
      std::string(sm_data::kActivateSubstateP2));
    RCLCPP_INFO(getLogger(), "ACTIVATE::P1 - moving to waypoint 1 (both arms)");
  }
};

// ---- P2: final approach waypoint -------------------------------------------
struct StActivateP2 : smacc2::SmaccState<StActivateP2, StActivate>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<smacc2::EvCbSuccess<CbMoveModeKnownState, OrBothArms>, StDelay>,
    smacc2::Transition<smacc2::EvCbFailure<CbMoveModeKnownState, OrBothArms>, StPause>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrBothArms, CbMoveModeKnownState>(
      "je_arm_pcb_inspection_sm",
      "dual_pick_p2.yaml");
  }

  void onEntry()
  {
    // Mark current sub-state; delay will mark activate as done -> EvCanWork.
    this->setGlobalSMData(
      std::string(sm_data::kActivateResumeSubstateId),
      std::string(sm_data::kActivateSubstateP2));
    this->setGlobalSMData(
      std::string(sm_data::kActivateDelayNextSubstateId),
      std::string(sm_data::kActivateSubstateDone));
    RCLCPP_INFO(getLogger(), "ACTIVATE::P2 - moving to waypoint 2 (both arms)");
  }
};

}  // namespace je_arm_pcb_inspection_sm
