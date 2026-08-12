#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/logging.hpp"

namespace je_arm_pcb_inspection_sm
{

struct SmJeArmPcbInspection;

struct StActivate;
struct StWaitResources;
struct StTestGripper;
struct StWork;
struct StBack;
struct StBackToIdle;

/// PAUSE 恢复路由状态：根据黑板中的 resume_state_id 恢复到目标状态
struct StPauseResumeRouter : smacc2::SmaccState<StPauseResumeRouter, SmJeArmPcbInspection>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvResumeToActivate, StActivate>,
    smacc2::Transition<EvResumeToWaitResources, StWaitResources>,
    smacc2::Transition<EvResumeToTestGripper, StTestGripper>,
    smacc2::Transition<EvResumeToWork, StWork>,
    smacc2::Transition<EvResumeToBack, StBack>,
    smacc2::Transition<EvResumeToBackToIdle, StBackToIdle>
  > reactions;

  void onEntry()
  {
    std::string resumeState = sm_data::kWaitResourcesState;
    this->getGlobalSMData(std::string(sm_data::kResumeStateId), resumeState);
    this->setGlobalSMData(std::string(sm_data::kResumeFromPause), false);

    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] TRANSITION PAUSE --(resume)--> %s",
      log_utils::bjtNowString().c_str(),
      resumeState.c_str());
    if (resumeState == sm_data::kActivateState)
    {
      this->setGlobalSMData(std::string(sm_data::kActivateResumeFromPause), true);
      this->template postEvent<EvResumeToActivate>();
    }
    else if (resumeState == sm_data::kTestGripperState)
    {
      this->template postEvent<EvResumeToTestGripper>();
    }
    else if (resumeState == sm_data::kWorkState)
    {
      this->setGlobalSMData(std::string(sm_data::kResumeFromPause), true);
      this->template postEvent<EvResumeToWork>();
    }
    else if (resumeState == sm_data::kBackState)
    {
      this->template postEvent<EvResumeToBack>();
    }
    else if (resumeState == sm_data::kBackToIdleState)
    {
      this->setGlobalSMData(std::string(sm_data::kBackToIdleResumeFromPause), true);
      this->template postEvent<EvResumeToBackToIdle>();
    }
    else
    {
      this->template postEvent<EvResumeToWaitResources>();
    }

  }

  void onExit()
  {
    RCLCPP_DEBUG(log_utils::bizLogger(), "[%s] EXIT StPauseResumeRouter", log_utils::bjtNowString().c_str());
  }
};

}  // namespace je_arm_pcb_inspection_sm