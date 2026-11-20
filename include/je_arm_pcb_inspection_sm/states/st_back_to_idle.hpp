#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "je_arm_pcb_inspection_sm/behaviors/cb_move_mode_known_state.hpp"
#include "je_arm_pcb_inspection_sm/components/cp_top_level_flow.hpp"
#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/logging.hpp"

namespace je_arm_pcb_inspection_sm
{

struct SmJeArmPcbInspection;
struct StIdle;
struct StPause;

struct StBackToIdleResumeRouter;
struct StBackToIdleP2;
struct StBackToIdleP1;

// ---- Composite parent -------------------------------------------------------
struct StBackToIdle : smacc2::SmaccState<StBackToIdle, SmJeArmPcbInspection, StBackToIdleResumeRouter>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvPauseRequested, StPause>,
    smacc2::Transition<EvBackToIdleDone, StIdle>
  > reactions;

  void onEntry()
  {
    this->requiresComponent(flow_);
    flow_->setResumeTarget(sm_data::kBackToIdleState);

    // Fresh entry: reset sub-state pointer to P2.
    // Resume from pause: preserve the saved sub-state.
    bool resumeFromPause = false;
    this->getGlobalSMData(std::string(sm_data::kBackToIdleResumeFromPause), resumeFromPause);
    this->setGlobalSMData(std::string(sm_data::kBackToIdleResumeFromPause), false);
    if (!resumeFromPause)
    {
      this->setGlobalSMData(
        std::string(sm_data::kBackToIdleResumeSubstateId),
        std::string(sm_data::kBackToIdleSubstateP2));
    }

    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] ENTER BACK_TO_IDLE (reverse activate)",
      log_utils::bjtNowString().c_str());
  }

private:
  CpTopLevelFlow * flow_;
};

// ---- Resume router ----------------------------------------------------------
struct StBackToIdleResumeRouter : smacc2::SmaccState<StBackToIdleResumeRouter, StBackToIdle>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvBackToIdleResumeToP2, StBackToIdleP2>,
    smacc2::Transition<EvBackToIdleResumeToP1, StBackToIdleP1>,
    smacc2::Transition<EvBackToIdleDone, StIdle>
  > reactions;

  void onEntry()
  {
    std::string substate = sm_data::kBackToIdleSubstateP2;
    this->getGlobalSMData(std::string(sm_data::kBackToIdleResumeSubstateId), substate);

    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] BACK_TO_IDLE::RESUME_ROUTER --> %s",
      log_utils::bjtNowString().c_str(),
      substate.c_str());

    if (substate == sm_data::kBackToIdleSubstateP1)
    {
      this->template postEvent<EvBackToIdleResumeToP1>();
    }
    else if (substate == sm_data::kBackToIdleSubstateDone)
    {
      this->template postEvent<EvBackToIdleDone>();
    }
    else
    {
      this->template postEvent<EvBackToIdleResumeToP2>();
    }
  }
};

// ---- P2: move to activate P1 pose ------------------------------------------
struct StBackToIdleP2 : smacc2::SmaccState<StBackToIdleP2, StBackToIdle>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<smacc2::EvCbSuccess<CbMoveModeKnownState, OrBothArms>, StBackToIdleP1>,
    smacc2::Transition<smacc2::EvCbFailure<CbMoveModeKnownState, OrBothArms>, StPause>>
    reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrBothArms, CbMoveModeKnownState>(
      "je_arm_pcb_inspection_sm",
      "dual_pick_p1.yaml");
  }

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kBackToIdleResumeSubstateId),
      std::string(sm_data::kBackToIdleSubstateP2));
    RCLCPP_INFO(getLogger(), "BACK_TO_IDLE::P2 - moving to activate P1 pose (both arms)");
  }
};

// ---- P1: move to home pose --------------------------------------------------
struct StBackToIdleP1 : smacc2::SmaccState<StBackToIdleP1, StBackToIdle>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<smacc2::EvCbSuccess<CbMoveModeKnownState, OrBothArms>, StIdle>,
    smacc2::Transition<smacc2::EvCbFailure<CbMoveModeKnownState, OrBothArms>, StPause>>
    reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrBothArms, CbMoveModeKnownState>(
      "je_arm_pcb_inspection_sm",
      "dual_back_to_idle_home.yaml");
  }

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kBackToIdleResumeSubstateId),
      std::string(sm_data::kBackToIdleSubstateP1));
    RCLCPP_INFO(getLogger(), "BACK_TO_IDLE::P1 - moving to home pose (both arms)");
  }
};

}  // namespace je_arm_pcb_inspection_sm
