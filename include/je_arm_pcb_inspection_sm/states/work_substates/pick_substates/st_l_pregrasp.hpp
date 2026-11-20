#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cl_moveit2z/client_behaviors/cb_move_end_effector_linear_seeded.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/pick_offset_loader.hpp"
#include "je_arm_pcb_inspection_sm/utils/logging.hpp"

namespace je_arm_pcb_inspection_sm
{

struct StPause;
struct StDelay;

namespace work_substates
{

struct StPick;

namespace pick_substates
{

/**
 * Reads PCB target pose from the blackboard and delegates planning/execution
 * to CbMoveEndEffectorLinearSeeded (linear Cartesian interpolation + seeded IK).
 */
class CbMovePcbTargetPose : public cl_moveit2z::CbMoveEndEffectorLinearSeeded
{
public:
  CbMovePcbTargetPose() = default;

  void onEntry() override
  {
    tip_link_ = getLeftTipLink(this->getNode());

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.orientation.w = 1.0;

    auto * stateMachine = this->getStateMachine();
    stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetFrameId), pose.header.frame_id);
    stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetX), pose.pose.position.x);
    stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetY), pose.pose.position.y);
    stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetZ), pose.pose.position.z);
    stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetQx), pose.pose.orientation.x);
    stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetQy), pose.pose.orientation.y);
    stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetQz), pose.pose.orientation.z);
    stateMachine->getGlobalSMData(std::string(sm_data::kPcbTargetQw), pose.pose.orientation.w);

    double offsetX = 0.0;
    double offsetY = 0.0;
    double offsetZ = 0.0;
    std::string offsetPath;
    const bool offsetLoaded =
      je_arm_pcb_inspection_sm::utils::loadPickOffset(offsetX, offsetY, offsetZ, offsetPath);

    pose.pose.position.x += offsetX;
    pose.pose.position.y += offsetY;
    pose.pose.position.z += offsetZ;
    targetPose = pose;

    this->requiresClient(movegroupClient_);
    auto & moveGroup = *(movegroupClient_->moveGroupClientInterface);
    moveGroup.startStateMonitor(1.0);
    const auto currentState = moveGroup.getCurrentState(1.0);
    if (currentState)
    {
      moveGroup.setStartState(*currentState);
    }
    else
    {
      RCLCPP_WARN(
        getLogger(),
        "[CbMovePcbTargetPose] MoveIt current state unavailable before pregrasp planning");
    }

    RCLCPP_INFO(
      getLogger(),
      "[CbMovePcbTargetPose] PREGRASP target (pick_offset: %.4f, %.4f, %.4f, file=%s) -> frame=%s pos=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
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

    // Delegate linear-cartesian seeded planning + execution to CL
    CbMoveEndEffectorLinearSeeded::onEntry();
  }
};

struct StGripperOpen;

// Forward declaration for the single pregrasp motion sub-state
struct StLPregraspMove;

// ---- Composite parent: StLPregrasp ----------------------------------------
// P1 and P2 waypoints have been moved to StActivate (runs once at startup).
// StLPregrasp now only has one motion state (to final pregrasp pose).
// EvPauseRequested is NOT listed here; it bubbles up to StPick which handles it.
struct StLPregrasp : smacc2::SmaccState<StLPregrasp, StPick, StLPregraspMove>
{
  using SmaccState::SmaccState;

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(),
      "WORK::PICK::L_PREGRASP - moving to pregrasp pose");
  }
};

// ---- single move: final pregrasp pose -------------------------------------
struct StLPregraspMove : smacc2::SmaccState<StLPregraspMove, StLPregrasp>
{
  using SmaccState::SmaccState;

  // EvAtPregrasp: manual bypass key ('n') for testing/debugging
  typedef boost::mpl::list<
    smacc2::Transition<smacc2::EvCbSuccess<CbMovePcbTargetPose, OrArm>, StDelay>,
    smacc2::Transition<smacc2::EvCbFailure<CbMovePcbTargetPose, OrArm>, StPause>,
    smacc2::Transition<EvAtPregrasp, StGripperOpen>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrArm, CbMovePcbTargetPose>();
  }

  void onEntry()
  {
    this->setGlobalSMData(
      std::string(sm_data::kPickResumeSubstateId),
      std::string(sm_data::kPickSubstateLPregrasp));
    this->setGlobalSMData(
      std::string(sm_data::kPickDelayNextSubstateId),
      std::string(sm_data::kPickSubstateGripperOpen));
    this->setGlobalSMData(
      std::string(sm_data::kResumeStateId),
      std::string(sm_data::kWorkState));
    this->setGlobalSMData(
      std::string(sm_data::kWorkResumeSubstateId),
      std::string(sm_data::kWorkSubstatePick));
    this->setGlobalSMData(
      std::string(sm_data::kWorkDelayNextSubstateId),
      std::string(sm_data::kWorkSubstatePick));
    RCLCPP_INFO(
      getLogger(),
      "WORK::PICK::L_PREGRASP::MOVE - linear move to simulated PCBA end-effector pose [bypass: 'n']");
  }
};

}  // namespace pick_substates

}  // namespace work_substates

}  // namespace je_arm_pcb_inspection_sm
