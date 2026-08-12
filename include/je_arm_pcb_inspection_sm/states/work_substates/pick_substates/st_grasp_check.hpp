#pragma once

#include <chrono>
#include <cmath>

#include <common/msg/oculus_init_joint_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/gripper_command_loader.hpp"
#include "je_arm_pcb_inspection_sm/utils/logging.hpp"

namespace je_arm_pcb_inspection_sm
{

namespace work_substates
{

struct StPick;

namespace pick_substates
{

/// Checks the configured closing position after retreat: reaching it means no PCB blocks the gripper.
struct StGraspCheck : smacc2::SmaccState<StGraspCheck, StPick>
{
  using SmaccState::SmaccState;

  void onEntry()
  {
    pickCloseConfig_ = utils::loadGripperCommandConfig("pick_close");
    decisionPosted_ = false;
    this->setGlobalSMData(
      std::string(sm_data::kPickResumeSubstateId),
      std::string(sm_data::kPickSubstateLRetreat));

    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] WORK::PICK::GRASP_CHECK waiting for gripper feedback after retreat delay | target=%.4f tolerance=%.4f left=%d right=%d",
      log_utils::bjtNowString().c_str(),
      pickCloseConfig_.position,
      kPositionTolerance,
      pickCloseConfig_.leftValid,
      pickCloseConfig_.rightValid);
    startFeedbackCheck();
  }

  void onExit()
  {
    if (timeoutTimer_)
    {
      timeoutTimer_->cancel();
      timeoutTimer_.reset();
    }
    feedbackSub_.reset();
  }

private:
  static constexpr double kPositionTolerance = 0.03;

  void startFeedbackCheck()
  {
    if (decisionPosted_ || feedbackSub_)
    {
      return;
    }

    feedbackSub_ = getNode()->create_subscription<common::msg::OculusInitJointState>(
      "/joint_states_double_arm",
      rclcpp::QoS(10).reliable(),
      [this](const common::msg::OculusInitJointState::SharedPtr msg)
      {
        onFeedback(msg);
      });

    timeoutTimer_ = getNode()->create_wall_timer(
      std::chrono::seconds(1),
      [this]()
      {
        if (decisionPosted_)
        {
          return;
        }

        RCLCPP_WARN(
          log_utils::bizLogger(),
          "[%s] WORK::PICK::GRASP_CHECK feedback timeout; continuing to INSPECT",
          log_utils::bjtNowString().c_str());
        postPickDone();
      });
  }

  void onFeedback(const common::msg::OculusInitJointState::SharedPtr & msg)
  {
    if (!msg || decisionPosted_)
    {
      return;
    }

    const bool leftAtCloseTarget = !pickCloseConfig_.leftValid ||
      (msg->left_valid && std::fabs(msg->left_gripper - pickCloseConfig_.position) <= kPositionTolerance);
    const bool rightAtCloseTarget = !pickCloseConfig_.rightValid ||
      (msg->right_valid && std::fabs(msg->right_gripper - pickCloseConfig_.position) <= kPositionTolerance);

    if (leftAtCloseTarget && rightAtCloseTarget)
    {
      decisionPosted_ = true;
      RCLCPP_WARN(
        log_utils::bizLogger(),
        "[%s] WORK::PICK::GRASP_CHECK gripper reached empty-grasp target %.4f | left=%.4f right=%.4f; retrying PICK",
        log_utils::bjtNowString().c_str(),
        pickCloseConfig_.position,
        msg->left_gripper,
        msg->right_gripper);
      this->setGlobalSMData(std::string(sm_data::kPickResumeFromPause), false);
      // Let StPick perform its normal self-transition and restart from L_PREGRASP.
      this->template postEvent<EvPickRetry>();
      return;
    }

    decisionPosted_ = true;
    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] WORK::PICK::GRASP_CHECK PCB grasp confirmed | target=%.4f left=%.4f right=%.4f",
      log_utils::bjtNowString().c_str(),
      pickCloseConfig_.position,
      msg->left_gripper,
      msg->right_gripper);
    // Let StPick own the normal PICK -> INSPECT transition.
    postPickDone();
  }

  void postPickDone()
  {
    decisionPosted_ = true;
    this->template postEvent<EvPickDone>();
  }

  utils::GripperCommandConfig pickCloseConfig_;
  bool decisionPosted_{false};
  rclcpp::Subscription<common::msg::OculusInitJointState>::SharedPtr feedbackSub_;
  rclcpp::TimerBase::SharedPtr timeoutTimer_;
};

}  // namespace pick_substates

}  // namespace work_substates

}  // namespace je_arm_pcb_inspection_sm