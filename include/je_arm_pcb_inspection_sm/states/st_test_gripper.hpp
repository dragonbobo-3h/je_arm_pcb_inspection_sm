#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <common/msg/oculus_init_joint_state.hpp>
#include <je_software/msg/end_effector_command_lr.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/test_gripper_config_loader.hpp"
#include "je_arm_pcb_inspection_sm/utils/logging.hpp"

namespace je_arm_pcb_inspection_sm
{

struct StIdle;
struct StPause;

/// TEST_GRIPPER 状态：测试夹爪功能
/// 从 config/test_gripper.yaml 直接加载夹爪参数（模式、位置、预设等）
/// 无需重新编译，修改 YAML 文件后立即生效
struct StTestGripper : smacc2::SmaccState<StTestGripper, SmJeArmPcbInspection>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvGripperClosed, StIdle>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  static void staticConfigure() {}

  void onEntry()
  {
    config_ = je_arm_pcb_inspection_sm::utils::loadTestGripperConfig();
    current_step_index_ = 0;
    step_completed_ = false;
    command_completed_ = false;
    received_feedback_ = false;
    last_left_valid_ = false;
    last_right_valid_ = false;
    last_left_gripper_ = 0.0;
    last_right_gripper_ = 0.0;

    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] ENTER TEST_GRIPPER - loaded %zu test step(s) from test_gripper.yaml | wait_feedback=%d, topic='%s', feedback_topic='%s', tol=%.3f [bypass: 'n']",
      log_utils::bjtNowString().c_str(),
      config_.steps.size(),
      config_.waitForFeedback,
      config_.topic.c_str(),
      config_.feedbackTopic.c_str(),
      config_.positionTolerance);

    if (config_.delayBeforeSec > 0.0)
    {
      RCLCPP_WARN(
        log_utils::bizLogger(),
        "[%s] TEST_GRIPPER delay_before_sec=%.2f is ignored because the test sequence starts immediately on state entry.",
        log_utils::bjtNowString().c_str(),
        config_.delayBeforeSec);
    }

    if (!publisher_)
    {
      publisher_ = getNode()->create_publisher<je_software::msg::EndEffectorCommandLR>(
        config_.topic,
        rclcpp::QoS(10).reliable());
    }

    if (!feedback_sub_)
    {
      feedback_sub_ = getNode()->create_subscription<common::msg::OculusInitJointState>(
        config_.feedbackTopic,
        rclcpp::QoS(10).reliable(),
        [this](const common::msg::OculusInitJointState::SharedPtr msg)
        {
          onFeedback(msg);
        });
    }

    if (!watchdog_timer_)
    {
      watchdog_timer_ = getNode()->create_wall_timer(
        std::chrono::milliseconds(50),
        [this]()
        {
          onWatchdogTick();
        });
    }

    dispatchCurrentStep();
  }

  void onExit()
  {
    command_completed_ = true;
    step_completed_ = true;
    if (watchdog_timer_)
    {
      watchdog_timer_->cancel();
      watchdog_timer_.reset();
    }
    feedback_sub_.reset();
    publisher_.reset();

    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] EXIT TEST_GRIPPER",
      log_utils::bjtNowString().c_str());
  }

private:
  void dispatchCurrentStep()
  {
    if (current_step_index_ >= config_.steps.size())
    {
      command_completed_ = true;
      this->template postEvent<EvGripperClosed>();
      return;
    }

    const auto & step = config_.steps[current_step_index_];
    if (!step.leftValid && !step.rightValid)
    {
      RCLCPP_WARN(
        log_utils::bizLogger(),
        "[%s] TEST_GRIPPER step[%zu] '%s' has neither left nor right enabled",
        log_utils::bjtNowString().c_str(),
        current_step_index_,
        step.name.c_str());
      this->template postEvent<EvPauseRequested>();
      return;
    }

    je_software::msg::EndEffectorCommandLR msg;
    msg.left_valid = step.leftValid;
    msg.right_valid = step.rightValid;

    auto fill_command = [&](je_software::msg::EndEffectorCommand & command)
    {
      command.mode = step.mode;
      command.position = step.position;
      command.preset = step.preset;
      command.command = step.command;
      command.torque = step.torque;
    };

    if (step.leftValid)
    {
      fill_command(msg.left);
    }
    if (step.rightValid)
    {
      fill_command(msg.right);
    }

    publisher_->publish(msg);
    step_started_at_ = getNode()->now();
    step_completed_ = false;

    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] TEST_GRIPPER step[%zu/%zu] '%s' dispatched | mode=%d pos=%.3f preset=%d command=%s torque=%.3f left=%d right=%d hold=%.2fs timeout=%.2fs",
      log_utils::bjtNowString().c_str(),
      current_step_index_ + 1,
      config_.steps.size(),
      step.name.c_str(),
      step.mode,
      step.position,
      step.preset,
      step.command.c_str(),
      step.torque,
      step.leftValid,
      step.rightValid,
        step.minDurationSec,
      step.timeoutSec);
  }

  void onFeedback(const common::msg::OculusInitJointState::SharedPtr msg)
  {
    if (!msg || command_completed_ || step_completed_ || current_step_index_ >= config_.steps.size())
    {
      return;
    }

    const auto & step = config_.steps[current_step_index_];
    const auto elapsed = (getNode()->now() - step_started_at_).seconds();
    received_feedback_ = true;
    last_left_valid_ = msg->left_valid;
    last_right_valid_ = msg->right_valid;
    last_left_gripper_ = msg->left_gripper;
    last_right_gripper_ = msg->right_gripper;
    const bool leftOk = !step.leftValid || (msg->left_valid && isTargetReached(step, msg->left_gripper));
    const bool rightOk = !step.rightValid || (msg->right_valid && isTargetReached(step, msg->right_gripper));

    if (leftOk && rightOk && elapsed >= step.minDurationSec)
    {
      RCLCPP_INFO(
        log_utils::bizLogger(),
        "[%s] TEST_GRIPPER step[%zu/%zu] '%s' reached target after %.2fs | left=%.4f right=%.4f",
        log_utils::bjtNowString().c_str(),
        current_step_index_ + 1,
        config_.steps.size(),
        step.name.c_str(),
        elapsed,
        msg->left_gripper,
        msg->right_gripper);

      step_completed_ = true;
      ++current_step_index_;
      dispatchCurrentStep();
    }
  }

  void onWatchdogTick()
  {
    if (command_completed_ || step_completed_ || current_step_index_ >= config_.steps.size())
    {
      return;
    }

    const auto & step = config_.steps[current_step_index_];
    const auto elapsed = (getNode()->now() - step_started_at_).seconds();
    if (elapsed <= step.timeoutSec)
    {
      return;
    }

    if (!config_.waitForFeedback || step.mode == je_software::msg::EndEffectorCommand::MODE_PRESET)
    {
      RCLCPP_WARN(
        log_utils::bizLogger(),
        "[%s] TEST_GRIPPER step[%zu/%zu] '%s' timed out after %.2fs; treating as success fallback",
        log_utils::bjtNowString().c_str(),
        current_step_index_ + 1,
        config_.steps.size(),
        step.name.c_str(),
        elapsed);
      step_completed_ = true;
      ++current_step_index_;
      dispatchCurrentStep();
      return;
    }

    const double target = step.position;
    const double feedback = step.leftValid ? last_left_gripper_ : last_right_gripper_;
    const bool feedbackValid = step.leftValid ? last_left_valid_ : last_right_valid_;
    RCLCPP_WARN(
      log_utils::bizLogger(),
      "[%s] TEST_GRIPPER step[%zu/%zu] '%s' timed out after %.2fs; posting pause | received_feedback=%d target=%.4f feedback=%.4f feedback_valid=%d diff=%.4f left=(valid=%d,pos=%.4f) right=(valid=%d,pos=%.4f)",
      log_utils::bjtNowString().c_str(),
      current_step_index_ + 1,
      config_.steps.size(),
      step.name.c_str(),
      elapsed,
      received_feedback_,
      target,
      feedback,
      feedbackValid,
      std::fabs(feedback - target),
      last_left_valid_,
      last_left_gripper_,
      last_right_valid_,
      last_right_gripper_);
    command_completed_ = true;
    this->template postEvent<EvPauseRequested>();
  }

  bool isTargetReached(const utils::TestGripperStep & step, double feedbackValue) const
  {
    if (
      step.mode == je_software::msg::EndEffectorCommand::MODE_POSITION ||
      step.mode == je_software::msg::EndEffectorCommand::MODE_TORQUE)
    {
      return std::fabs(feedbackValue - step.position) <= config_.positionTolerance;
    }
    return std::isfinite(feedbackValue);
  }

  utils::TestGripperConfig config_;
  size_t current_step_index_{0};
  bool step_completed_{false};
  bool command_completed_{false};
  bool received_feedback_{false};
  bool last_left_valid_{false};
  bool last_right_valid_{false};
  double last_left_gripper_{0.0};
  double last_right_gripper_{0.0};
  rclcpp::Time step_started_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<je_software::msg::EndEffectorCommandLR>::SharedPtr publisher_;
  rclcpp::Subscription<common::msg::OculusInitJointState>::SharedPtr feedback_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace je_arm_pcb_inspection_sm


