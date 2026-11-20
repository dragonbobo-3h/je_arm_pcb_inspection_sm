#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cl_moveit2z/client_behaviors/cb_move_end_effector_linear_seeded.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/pick_offset_loader.hpp"

namespace je_arm_pcb_inspection_sm
{

struct StPause;
struct StDelay;

namespace work_substates
{

struct StPick;

namespace pick_substates
{

struct StLRetreat;

struct StCartesianUp : smacc2::SmaccState<StCartesianUp, StPick>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<
      smacc2::EvCbSuccess<cl_moveit2z::CbMoveEndEffectorLinearSeeded, OrArm>,
      StDelay>,
    smacc2::Transition<
      smacc2::EvCbFailure<cl_moveit2z::CbMoveEndEffectorLinearSeeded, OrArm>,
      StPause>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal_runtime<OrArm, cl_moveit2z::CbMoveEndEffectorLinearSeeded>(
      [](cl_moveit2z::CbMoveEndEffectorLinearSeeded & bh, StCartesianUp & state)
      {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "base_link";
        pose.pose.orientation.w = 1.0;

        state.getGlobalSMData(std::string(sm_data::kPcbTargetFrameId), pose.header.frame_id);
        state.getGlobalSMData(std::string(sm_data::kPcbTargetX), pose.pose.position.x);
        state.getGlobalSMData(std::string(sm_data::kPcbTargetY), pose.pose.position.y);
        state.getGlobalSMData(std::string(sm_data::kPcbTargetZ), pose.pose.position.z);
        state.getGlobalSMData(std::string(sm_data::kPcbTargetQx), pose.pose.orientation.x);
        state.getGlobalSMData(std::string(sm_data::kPcbTargetQy), pose.pose.orientation.y);
        state.getGlobalSMData(std::string(sm_data::kPcbTargetQz), pose.pose.orientation.z);
        state.getGlobalSMData(std::string(sm_data::kPcbTargetQw), pose.pose.orientation.w);

        double offsetX = 0.0;
        double offsetY = 0.0;
        double offsetZ = 0.0;
        std::string offsetPath;
        je_arm_pcb_inspection_sm::utils::loadPickOffset(offsetX, offsetY, offsetZ, offsetPath);

        pose.pose.position.x += offsetX;
        pose.pose.position.y += offsetY;
        pose.pose.position.z += offsetZ;

        bh.tip_link_ = getLeftTipLink(state.getNode());
        bh.targetPose = pose;

        RCLCPP_INFO(
          state.getLogger(),
          "WORK::PICK::CARTESIAN_UP - runtime configured target using pick_offset: frame=%s pos=(%.4f, %.4f, %.4f)",
          bh.targetPose.header.frame_id.c_str(),
          bh.targetPose.pose.position.x,
          bh.targetPose.pose.position.y,
          bh.targetPose.pose.position.z);
      });
  }

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kPickResumeSubstateId),
      std::string(sm_data::kPickSubstateCartesianUp));
    this->setGlobalSMData(
      std::string(sm_data::kPickDelayNextSubstateId),
      std::string(sm_data::kPickSubstateLRetreat));
    this->setGlobalSMData(
      std::string(sm_data::kResumeStateId),
      std::string(sm_data::kWorkState));
    this->setGlobalSMData(
      std::string(sm_data::kWorkResumeSubstateId),
      std::string(sm_data::kWorkSubstatePick));
    this->setGlobalSMData(
      std::string(sm_data::kWorkDelayNextSubstateId),
      std::string(sm_data::kWorkSubstatePick));
    RCLCPP_INFO(getLogger(), "WORK::PICK::CARTESIAN_UP - executing linear seeded up motion using pick_offset");
  }
};

}  // namespace pick_substates

}  // namespace work_substates

}  // namespace je_arm_pcb_inspection_sm
