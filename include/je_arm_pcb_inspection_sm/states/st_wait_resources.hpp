#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include <common/msg/oculus_init_joint_state.hpp>
#include <je_software/msg/end_effector_command_lr.hpp>
#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>
#include "je_arm_pcb_inspection_sm/components/cp_top_level_flow.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/gripper_command_loader.hpp"
#include "je_arm_pcb_inspection_sm/utils/logging.hpp"

namespace je_arm_pcb_inspection_sm
{

namespace detail
{
inline bool getBoolParameter(
  const rclcpp::Node::SharedPtr & node, const std::string & name, bool defaultValue)
{
  rclcpp::Parameter parameter;
  if (!node->get_parameter(name, parameter))
  {
    return defaultValue;
  }

  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
  {
    return parameter.as_bool();
  }

  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
  {
    auto value = parameter.as_string();
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    if (value == "true" || value == "1" || value == "yes" || value == "on")
    {
      return true;
    }
    if (value == "false" || value == "0" || value == "no" || value == "off")
    {
      return false;
    }
  }

  return defaultValue;
}

inline double getDoubleParameter(
  const rclcpp::Node::SharedPtr & node, const std::string & name, double defaultValue)
{
  rclcpp::Parameter parameter;
  if (!node->get_parameter(name, parameter))
  {
    return defaultValue;
  }

  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
  {
    return parameter.as_double();
  }

  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
  {
    return static_cast<double>(parameter.as_int());
  }

  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
  {
    try
    {
      return std::stod(parameter.as_string());
    }
    catch (const std::exception &)
    {
      return defaultValue;
    }
  }

  return defaultValue;
}
}  // namespace detail

struct SmJeArmPcbInspection;

// 前向声明
struct StBackToIdle;
struct StWork;
struct StDelay;
struct StPause;

/// WAIT_RESOURCES 状态：复用的资源等待入口。
/// - 进入 PICK 前：等待 PCB + 可用 bin
/// - INSPECT 结束后：等待具体 place target
struct StWaitResources : smacc2::SmaccState<StWaitResources, SmJeArmPcbInspection>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvCanWork, StWork>,
    smacc2::Transition<EvWaitTimeout, StBackToIdle>,
    smacc2::Transition<EvBackToIdleRequested, StBackToIdle>,
    smacc2::Transition<EvDelayRequested, StDelay>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void onEntry() 
  { 
    this->requiresComponent(flow_);
    auto node = this->getStateMachine().getNode();
    enable_gripper_control_ = detail::getBoolParameter(node, "enable_gripper_control", true);
    simulated_gripper_action_sec_ = detail::getDoubleParameter(node, "simulated_gripper_action_sec", 2.0);
    gripper_action_delay_sec_ = detail::getDoubleParameter(node, "gripper_action_delay_sec", 0.0);
    this->getGlobalSMData(std::string(sm_data::kWorkResumeSubstateId), pendingWorkSubstate_);
    enteredTime_ = std::chrono::steady_clock::now();
    gripperCloseConfirmedTime_ = enteredTime_;
    transitionPosted_ = false;
    gripperCloseConfirmed_ = false;
    flow_->setResumeTarget(sm_data::kWaitResourcesState);
    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] ENTER WAIT_RESOURCES | pending_work_substate=%s enable_gripper_control=%d simulated_gripper_action_sec=%.2f gripper_action_delay_sec=%.2f",
      log_utils::bjtNowString().c_str(),
      pendingWorkSubstate_.c_str(),
      enable_gripper_control_,
      simulated_gripper_action_sec_,
      gripper_action_delay_sec_);

    const auto gripperCfg = je_arm_pcb_inspection_sm::utils::loadGripperCommandConfig("dual_close");
    gripperCloseConfig_ = gripperCfg;

    if (!enable_gripper_control_)
    {
      RCLCPP_INFO(
        log_utils::bizLogger(),
        "[%s] WAIT_RESOURCES gripper control disabled, simulating dual-close for %.2fs",
        log_utils::bjtNowString().c_str(),
        getEffectiveGripperDelaySec());
    }

    if (enable_gripper_control_ && !gripperPublisher_)
    {
      gripperPublisher_ = node->create_publisher<je_software::msg::EndEffectorCommandLR>(
        gripperCloseConfig_.topic,
        rclcpp::QoS(10).reliable());
    }

    if (enable_gripper_control_ && !gripperFeedbackSub_)
    {
      gripperFeedbackSub_ = node->create_subscription<common::msg::OculusInitJointState>(
        "/joint_states_double_arm",
        rclcpp::QoS(10).reliable(),
        [this](const common::msg::OculusInitJointState::SharedPtr msg)
        {
          this->onGripperFeedback(msg);
        });
    }

    if (enable_gripper_control_)
    {
      publishDualCloseCommand();
      if (!gripperCloseConfig_.waitForFeedback)
      {
        gripperCloseConfirmed_ = true;
        gripperCloseConfirmedTime_ = std::chrono::steady_clock::now();
      }
    }

    checkTimer_ = node->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]()
      {
        if (transitionPosted_)
        {
          return;
        }

        if (!gripperCloseConfirmed_)
        {
          const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::steady_clock::now() - enteredTime_);
          if (!enable_gripper_control_ && elapsed.count() >= getEffectiveGripperDelaySec())
          {
            gripperCloseConfirmed_ = true;
            gripperCloseConfirmedTime_ = std::chrono::steady_clock::now();
          }

          if (gripperCloseConfirmed_)
          {
            return;
          }

          if (elapsed.count() >= kGripperCloseTimeoutSec + getEffectiveGripperDelaySec())
          {
            transitionPosted_ = true;
            if (checkTimer_)
            {
              checkTimer_->cancel();
            }
            RCLCPP_WARN(
              log_utils::bizLogger(),
              "[%s] WAIT_RESOURCES dual-close confirmation timed out after %.2fs",
              log_utils::bjtNowString().c_str(),
              elapsed.count());
            this->template postEvent<EvPauseRequested>();
          }
          return;
        }

        const auto closeSettledElapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::steady_clock::now() - gripperCloseConfirmedTime_);
        if (enable_gripper_control_ && closeSettledElapsed.count() < getEffectiveGripperDelaySec())
        {
          return;
        }

        const bool canWork = flow_->isWorkReady();
        if (canWork)
        {
          transitionPosted_ = true;
          if (checkTimer_)
          {
            checkTimer_->cancel();
          }
          RCLCPP_INFO(
            log_utils::bizLogger(),
            "[%s] TRANSITION WAIT_RESOURCES --(EvCanWork)--> WORK",
            log_utils::bjtNowString().c_str());
          this->template postEvent<EvCanWork>();
          return;
        }

        const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::steady_clock::now() - enteredTime_);
        if (elapsed.count() >= flow_->waitTimeoutSeconds())
        {
          transitionPosted_ = true;
          if (checkTimer_)
          {
            checkTimer_->cancel();
          }
          RCLCPP_WARN(
            log_utils::bizLogger(),
            "[%s] TRANSITION WAIT_RESOURCES --(EvWaitTimeout)--> BACK_TO_IDLE",
            log_utils::bjtNowString().c_str());
          this->template postEvent<EvWaitTimeout>();
        }
      });
  }

  void onExit() 
  { 
    if (checkTimer_)
    {
      checkTimer_->cancel();
      checkTimer_.reset();
    }
    gripperFeedbackSub_.reset();
    gripperPublisher_.reset();
    RCLCPP_DEBUG(log_utils::bizLogger(), "[%s] EXIT WAIT_RESOURCES", log_utils::bjtNowString().c_str());
  }

private:
  static constexpr double kGripperPositionClosed = 0.0;
  static constexpr double kGripperPositionTolerance = 0.03;
  static constexpr double kGripperCloseTimeoutSec = 2.0;

  double getEffectiveGripperDelaySec() const
  {
    if (gripper_action_delay_sec_ > 0.0)
    {
      return gripper_action_delay_sec_;
    }

    if (!enable_gripper_control_)
    {
      return simulated_gripper_action_sec_ > 0.0 ? simulated_gripper_action_sec_ : 2.0;
    }

    return 0.0;
  }

  void publishDualCloseCommand()
  {
    je_software::msg::EndEffectorCommandLR msg;
    msg.left_valid = true;
    msg.right_valid = true;

    auto fillCommand = [this](je_software::msg::EndEffectorCommand & command)
    {
      command.mode = gripperCloseConfig_.mode;
      command.position = gripperCloseConfig_.position;
      command.preset = gripperCloseConfig_.preset;
      command.command = gripperCloseConfig_.command;
      command.torque = gripperCloseConfig_.torque;
    };

    fillCommand(msg.left);
    fillCommand(msg.right);

    gripperPublisher_->publish(msg);
    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] WAIT_RESOURCES dual-close command dispatched on %s",
      log_utils::bjtNowString().c_str(),
      gripperCloseConfig_.topic.c_str());
  }

  void onGripperFeedback(const common::msg::OculusInitJointState::SharedPtr msg)
  {
    if (!msg || transitionPosted_ || gripperCloseConfirmed_)
    {
      return;
    }

    const bool leftClosed = msg->left_valid &&
      std::fabs(msg->left_gripper - kGripperPositionClosed) <= kGripperPositionTolerance;
    const bool rightClosed = msg->right_valid &&
      std::fabs(msg->right_gripper - kGripperPositionClosed) <= kGripperPositionTolerance;

    if (!leftClosed || !rightClosed)
    {
      return;
    }

    gripperCloseConfirmed_ = true;
    gripperCloseConfirmedTime_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] WAIT_RESOURCES confirmed both grippers closed | left=%.4f right=%.4f",
      log_utils::bjtNowString().c_str(),
      msg->left_gripper,
      msg->right_gripper);
  }

  CpTopLevelFlow * flow_;
  rclcpp::TimerBase::SharedPtr checkTimer_;
  rclcpp::Publisher<je_software::msg::EndEffectorCommandLR>::SharedPtr gripperPublisher_;
  rclcpp::Subscription<common::msg::OculusInitJointState>::SharedPtr gripperFeedbackSub_;
  std::chrono::steady_clock::time_point enteredTime_;
  std::chrono::steady_clock::time_point gripperCloseConfirmedTime_;
  bool transitionPosted_{false};
  bool gripperCloseConfirmed_{false};
  bool enable_gripper_control_{true};
  double simulated_gripper_action_sec_{2.0};
  double gripper_action_delay_sec_{0.0};
  std::string pendingWorkSubstate_{sm_data::kWorkSubstatePick};
  je_arm_pcb_inspection_sm::utils::GripperCommandConfig gripperCloseConfig_;
};

}  // namespace je_arm_pcb_inspection_sm
