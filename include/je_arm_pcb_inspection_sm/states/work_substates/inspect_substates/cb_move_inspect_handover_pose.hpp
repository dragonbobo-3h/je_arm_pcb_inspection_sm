#pragma once

#include <stdexcept>
#include <string>

#include <cl_moveit2z/cl_moveit2z.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cl_moveit2z/client_behaviors/cb_move_end_effector_linear_seeded.hpp>

#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/inspect_handover_offset_loader.hpp"

namespace je_arm_pcb_inspection_sm
{
namespace work_substates
{
namespace inspect_substates
{

class CbMoveInspectHandoverPose : public cl_moveit2z::CbMoveEndEffectorLinearSeeded
{
public:
  CbMoveInspectHandoverPose(const std::string & offsetKey, const std::string & tipLink)
  : offsetKey_(offsetKey)
  {
    tip_link_ = tipLink;
    planningTimeSec_ = 1.5;
    linearStepMeters_ = 0.005;
    minPathFraction_ = 0.98;
  }

  void onEntry() override
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.orientation.w = 1.0;

    auto * stateMachine = this->getStateMachine();
    std::string anchorDescription = "pcb_target";

    if (offsetKey_ == "right_receive")
    {
      pose = getCurrentPoseFromOrthogonal<je_arm_pcb_inspection_sm::OrRightArm>("Link27");
      anchorDescription = "right_arm_current_pose";
    }
    else if (offsetKey_ == "left_receive")
    {
      pose = getCurrentPoseFromOrthogonal<je_arm_pcb_inspection_sm::OrLeftArm>("Link17");
      anchorDescription = "left_arm_current_pose";
    }
    else if (offsetKey_ == "right_retreat")
    {
      pose = getCurrentPoseFromOrthogonal<je_arm_pcb_inspection_sm::OrRightArm>("Link27");
      anchorDescription = "right_arm_current_pose";
    }
    else if (offsetKey_ == "left_retreat")
    {
      pose = getCurrentPoseFromOrthogonal<je_arm_pcb_inspection_sm::OrLeftArm>("Link17");
      anchorDescription = "left_arm_current_pose";
    }
    else
    {
      stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetFrameId), pose.header.frame_id);
      stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetX), pose.pose.position.x);
      stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetY), pose.pose.position.y);
      stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetZ), pose.pose.position.z);
      stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetQx), pose.pose.orientation.x);
      stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetQy), pose.pose.orientation.y);
      stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetQz), pose.pose.orientation.z);
      stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetQw), pose.pose.orientation.w);
    }

    double offsetX = 0.0;
    double offsetY = 0.0;
    double offsetZ = 0.0;
    std::string offsetPath;
    const bool offsetLoaded = je_arm_pcb_inspection_sm::utils::loadInspectHandoverOffset(
      offsetKey_, offsetX, offsetY, offsetZ, offsetPath);

    pose.pose.position.x += offsetX;
    pose.pose.position.y += offsetY;
    pose.pose.position.z += offsetZ;
    targetPose = pose;

    RCLCPP_INFO(
      getLogger(),
      "[CbMoveInspectHandoverPose] key=%s anchor=%s offset=(%.4f, %.4f, %.4f, file=%s) -> frame=%s pos=(%.4f, %.4f, %.4f)",
      offsetKey_.c_str(),
      anchorDescription.c_str(),
      offsetX,
      offsetY,
      offsetZ,
      offsetLoaded ? offsetPath.c_str() : "<default>",
      targetPose.header.frame_id.c_str(),
      targetPose.pose.position.x,
      targetPose.pose.position.y,
      targetPose.pose.position.z);

    CbMoveEndEffectorLinearSeeded::onEntry();
  }

private:
  template <typename TOrthogonal>
  geometry_msgs::msg::PoseStamped getCurrentPoseFromOrthogonal(const std::string & tipLink)
  {
    auto * stateMachine = this->getStateMachine();
    auto * orthogonal = stateMachine->template getOrthogonal<TOrthogonal>();
    if (orthogonal == nullptr)
    {
      throw std::runtime_error("inspect handover pose anchor orthogonal unavailable");
    }

    cl_moveit2z::ClMoveit2z * moveitClient = nullptr;
    if (!orthogonal->requiresClient(moveitClient) || moveitClient == nullptr ||
      moveitClient->moveGroupClientInterface == nullptr)
    {
      throw std::runtime_error("inspect handover pose anchor moveit client unavailable");
    }

    return moveitClient->moveGroupClientInterface->getCurrentPose(tipLink);
  }

  std::string offsetKey_;
};

}  // namespace inspect_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm