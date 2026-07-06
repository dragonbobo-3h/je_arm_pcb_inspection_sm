#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cl_moveit2z/client_behaviors/cb_move_known_state.hpp>

#include "je_arm_pcb_inspection_sm/components/cp_business_key_mapper.hpp"
#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"

namespace je_arm_pcb_inspection_sm
{

struct StPause;

namespace work_substates
{

struct StInspect;

namespace inspect_substates
{

struct StInspectFrontPoseMove;
struct StInspectFrontPoseWaitAck;
struct StInspectAlignForRightHandover;

struct StInspectFrontPose : smacc2::SmaccState<StInspectFrontPose, StInspect, StInspectFrontPoseMove>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvInspectStepAck, StInspectAlignForRightHandover>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void onEntry()
  {
    this->setGlobalSMData(std::string(sm_data::kInspectResumeSubstateId), std::string(sm_data::kInspectSubstateFrontPose));
    this->setGlobalSMData(std::string(sm_data::kInspectDelayNextSubstateId), std::string(sm_data::kInspectSubstateAlignForRightHandover));
    this->setGlobalSMData(std::string(sm_data::kResumeStateId), std::string(sm_data::kWorkState));
    this->setGlobalSMData(std::string(sm_data::kWorkResumeSubstateId), std::string(sm_data::kWorkSubstateInspect));
    this->setGlobalSMData(std::string(sm_data::kWorkDelayNextSubstateId), std::string(sm_data::kWorkSubstateInspect));
    RCLCPP_INFO(getLogger(), "WORK::INSPECT::FRONT_POSE - move to front inspect pose, inspect front side, then press 'n' to continue");
  }
};

struct StInspectFrontPoseMove : smacc2::SmaccState<StInspectFrontPoseMove, StInspectFrontPose>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<smacc2::EvCbSuccess<cl_moveit2z::CbMoveKnownState, OrArm>, StInspectFrontPoseWaitAck>,
    smacc2::Transition<smacc2::EvCbFailure<cl_moveit2z::CbMoveKnownState, OrArm>, StPause>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrArm, cl_moveit2z::CbMoveKnownState>(
      "je_arm_pcb_inspection_sm",
      "config/move_group_client/joint_states/inspect.yaml");
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "WORK::INSPECT::FRONT_POSE::MOVE - moving left arm to front-side inspect pose");
  }
};

struct StInspectFrontPoseWaitAck : smacc2::SmaccState<StInspectFrontPoseWaitAck, StInspectFrontPose>
{
  using SmaccState::SmaccState;

  void onEntry()
  {
    CpBusinessKeyMapper * keyMapper = nullptr;
    this->requiresComponent(keyMapper);
    if (keyMapper != nullptr)
    {
      keyMapper->request_front_inspection(this->getClassName());
    }

    RCLCPP_INFO(
      getLogger(),
      "WORK::INSPECT::FRONT_POSE::WAIT_ACK - front side is ready for inspection, attempting external ROS trigger; press 'n' to continue manually if needed");
  }
};

}  // namespace inspect_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm