#pragma once

#include <chrono>

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "je_arm_pcb_inspection_sm/behaviors/cb_move_mode_known_state.hpp"
#include "je_arm_pcb_inspection_sm/components/cp_top_level_flow.hpp"
#include "je_arm_pcb_inspection_sm/behaviors/cb_return_to_start_linear.hpp"
#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/logging.hpp"

namespace je_arm_pcb_inspection_sm
{

struct SmJeArmPcbInspection;

// 前向声明
struct StWaitResources;
struct StDelay;
struct StPause;

/// BACK 状态：回到 WAIT_RESOURCES 的起始位姿（Activate P2）
struct StBack : smacc2::SmaccState<StBack, SmJeArmPcbInspection>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<smacc2::EvCbSuccess<CbMoveModeKnownState, OrArm>, StWaitResources>,
    smacc2::Transition<smacc2::EvCbFailure<CbMoveModeKnownState, OrArm>, StPause>,
    smacc2::Transition<smacc2::EvCbSuccess<CbReturnToStartLinear, OrArm>, StWaitResources>,
    smacc2::Transition<smacc2::EvCbFailure<CbReturnToStartLinear, OrArm>, StPause>,
    smacc2::Transition<EvDelayRequested, StDelay>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void runtimeConfigure()
  {
    this->getGlobalSMData(std::string(sm_data::kBackLinearReturn), linearReturn_);
    if (linearReturn_)
    {
      this->configure<OrArm, CbReturnToStartLinear>();
    }
    else
    {
      this->configure<OrArm, CbMoveModeKnownState>(
        "je_arm_pcb_inspection_sm",
        "dual_pick_p2.yaml");
    }
  }

  void onEntry() 
  { 
    this->requiresComponent(flow_);
    this->setGlobalSMData(
      std::string(sm_data::kWorkResumeSubstateId),
      std::string(sm_data::kWorkSubstatePick));
    flow_->setResumeTarget(sm_data::kBackState);
    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] ENTER BACK - moving to WAIT_RESOURCES start pose (pick_p2), motion=%s",
      log_utils::bjtNowString().c_str(),
      linearReturn_ ? "Cartesian linear" : "joint-space");

    auto node = this->getStateMachine().getNode();
    syncTimer_ = node->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]()
      {
        (void)flow_->isWorkReady();
      });
  }

  void onExit() 
  { 
    if (syncTimer_)
    {
      syncTimer_->cancel();
      syncTimer_.reset();
    }
    RCLCPP_DEBUG(log_utils::bizLogger(), "[%s] EXIT BACK", log_utils::bjtNowString().c_str());
  }

private:
  bool linearReturn_{false};
  CpTopLevelFlow * flow_;
  rclcpp::TimerBase::SharedPtr syncTimer_;
};

}  // namespace je_arm_pcb_inspection_sm
