#pragma once

#include <cmath>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cl_moveit2z/client_behaviors/cb_move_end_effector_linear_seeded.hpp>
#include <moveit/robot_state/robot_state.h>
#include <yaml-cpp/yaml.h>

namespace je_arm_pcb_inspection_sm
{

// Keep the same end-effector destination as BACK's joint-space pick_p2 target,
// but reach it with the existing Cartesian linear planner. Joint angles at the
// endpoint may differ because IK is seeded from the current robot state.
class CbReturnToStartLinear : public cl_moveit2z::CbMoveEndEffectorLinearSeeded
{
public:
  void onEntry() override
  {
    this->requiresClient(movegroupClient_);
    auto & moveGroup = *movegroupClient_->moveGroupClientInterface;
    try
    {
      const auto currentState = moveGroup.getCurrentState(1.0);
      if (!currentState)
      {
        throw std::runtime_error("current robot state is unavailable");
      }
      moveit::core::RobotState targetState(*currentState);
      const auto * group = targetState.getJointModelGroup(moveGroup.getName());
      tip_link_ = "Link17";
      if (!group || !targetState.getRobotModel()->hasLinkModel(tip_link_))
      {
        throw std::runtime_error("return planning group or tip link is unavailable");
      }

      const auto path = ament_index_cpp::get_package_share_directory("je_arm_pcb_inspection_sm") +
        "/config/move_group_client/joint_states/dual_pick_p2.yaml";
      const auto config = YAML::LoadFile(path);
      const auto joints = config["joint_states"];
      // Only use the controlled planning group's joints, just as the existing
      // joint-space BACK does; do not move the other arm to compute this target.
      for (const auto & name : group->getVariableNames())
      {
        if (!joints[name])
        {
          throw std::runtime_error("missing pick_p2 joint: " + name);
        }
        const double value = joints[name].as<double>();
        if (!std::isfinite(value))
        {
          throw std::runtime_error("non-finite pick_p2 joint: " + name);
        }
        targetState.setVariablePosition(name, value);
      }
      targetState.update();
      const auto & transform = targetState.getGlobalLinkTransform(tip_link_);
      const Eigen::Quaterniond orientation(transform.rotation());
      targetPose.header.frame_id = targetState.getRobotModel()->getModelFrame();
      targetPose.pose.position.x = transform.translation().x();
      targetPose.pose.position.y = transform.translation().y();
      targetPose.pose.position.z = transform.translation().z();
      targetPose.pose.orientation.x = orientation.x();
      targetPose.pose.orientation.y = orientation.y();
      targetPose.pose.orientation.z = orientation.z();
      targetPose.pose.orientation.w = orientation.w();
    }
    catch (const std::exception & error)
    {
      RCLCPP_ERROR(getLogger(), "Cannot configure linear return to pick_p2: %s", error.what());
      this->postFailureEvent();
      return;
    }

    // No joint-space fallback: a failed straight-line path must pause.
    cl_moveit2z::CbMoveEndEffectorLinearSeeded::onEntry();
  }
};

}  // namespace je_arm_pcb_inspection_sm