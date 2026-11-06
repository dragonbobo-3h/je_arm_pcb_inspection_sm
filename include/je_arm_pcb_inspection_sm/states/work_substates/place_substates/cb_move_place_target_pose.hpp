#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cl_moveit2z/client_behaviors/cb_move_end_effector_linear_seeded.hpp>

#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/components/cp_top_level_flow.hpp"
#include "je_arm_pcb_inspection_sm/utils/pick_offset_loader.hpp"
#include "je_arm_pcb_inspection_sm/utils/place_offset_loader.hpp"

namespace je_arm_pcb_inspection_sm
{
namespace work_substates
{
namespace place_substates
{

class CbMovePlaceTargetPose : public cl_moveit2z::CbMoveEndEffectorLinearSeeded
{
public:
  CbMovePlaceTargetPose(bool useYamlOffset = false) : useYamlOffset_(useYamlOffset)
  {
    tip_link_ = "Link17";
  }

  void onEntry() override
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = -0.3182978288;
    pose.pose.position.y = -0.3067010057;
    pose.pose.position.z = 0.4037989943;
    pose.pose.orientation.x = -0.01659836570155305;
    pose.pose.orientation.y = -0.9941145267183089;
    pose.pose.orientation.z = -0.02580021092886362;
    pose.pose.orientation.w = 0.10389971674502421;

    auto * stateMachine = this->getStateMachine();
    stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetFrameId), pose.header.frame_id);
    stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetX), pose.pose.position.x);
    stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetY), pose.pose.position.y);
    stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetZ), pose.pose.position.z);
    stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetQx), pose.pose.orientation.x);
    stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetQy), pose.pose.orientation.y);
    stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetQz), pose.pose.orientation.z);
    stateMachine->getGlobalSMData(std::string(sm_data::kPlaceTargetQw), pose.pose.orientation.w);

    bool returningToPick = false;
    stateMachine->getGlobalSMData(std::string(sm_data::kReturningToPick), returningToPick);
    if (returningToPick)
    {
      CpTopLevelFlow * flow = nullptr;
      this->requiresComponent(flow);
      if (!flow || !flow->getLoopPickPose(pose))
      {
        RCLCPP_ERROR(getLogger(), "RETURN_TO_PICK has no valid saved pickup pose; refusing motion");
        this->postFailureEvent();
        return;
      }
    }

    std::string offsetPath;

    double offsetX = 0.0;
    double offsetY = 0.0;
    double offsetZ = 0.0;
    const bool offsetLoaded =
      useYamlOffset_ &&
      (returningToPick ?
        je_arm_pcb_inspection_sm::utils::loadPickOffset(offsetX, offsetY, offsetZ, offsetPath) :
        je_arm_pcb_inspection_sm::utils::loadPlaceOffset(offsetX, offsetY, offsetZ, offsetPath));

    if (useYamlOffset_)
    {
      pose.pose.position.x += offsetX;
      pose.pose.position.y += offsetY;
      pose.pose.position.z += offsetZ;
    }

    targetPose = pose;

    RCLCPP_INFO(
      getLogger(),
      "[CbMovePlaceTargetPose] target (return_to_pick=%s, use_yaml_offset=%s, offset: %.4f, %.4f, %.4f, file=%s) -> frame=%s pos=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
      returningToPick ? "true" : "false",
      useYamlOffset_ ? "true" : "false",
      offsetX,
      offsetY,
      offsetZ,
      offsetLoaded ? offsetPath.c_str() : "<default>",
      targetPose.header.frame_id.c_str(),
      targetPose.pose.position.x,
      targetPose.pose.position.y,
      targetPose.pose.position.z,
      targetPose.pose.orientation.x,
      targetPose.pose.orientation.y,
      targetPose.pose.orientation.z,
      targetPose.pose.orientation.w);

    CbMoveEndEffectorLinearSeeded::onEntry();
  }

  bool useYamlOffset_ = false;
};

}  // namespace place_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm
