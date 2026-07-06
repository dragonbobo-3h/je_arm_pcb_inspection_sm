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

struct StInspectRightViewMoveBothArms;
struct StInspectRightViewWaitAck;
struct StInspectAlignForLeftHandover;

struct StInspectRightView : smacc2::SmaccState<StInspectRightView, StInspect, StInspectRightViewMoveBothArms>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvInspectStepAck, StInspectAlignForLeftHandover>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void onEntry()
  {
    this->setGlobalSMData(std::string(sm_data::kInspectResumeSubstateId), std::string(sm_data::kInspectSubstateRightView));
    this->setGlobalSMData(std::string(sm_data::kWorkResumeSubstateId), std::string(sm_data::kWorkSubstateInspect));
    RCLCPP_INFO(
      getLogger(),
      "WORK::INSPECT::RIGHT_VIEW - move both arms to right-inspect pose, then inspect back side and press 'n' to continue");
  }
};

struct StInspectRightViewMoveBothArms : smacc2::SmaccState<StInspectRightViewMoveBothArms, StInspectRightView>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<
      smacc2::EvCbSuccess<cl_moveit2z::CbMoveKnownState, OrBothArms>,
      StInspectRightViewWaitAck>,
    smacc2::Transition<
      smacc2::EvCbFailure<cl_moveit2z::CbMoveKnownState, OrBothArms>,
      StPause>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrBothArms, cl_moveit2z::CbMoveKnownState>(
      "je_arm_pcb_inspection_sm",
      "config/move_group_client/joint_states/dual_inspect_right_view.yaml");
  }

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(),
      "WORK::INSPECT::RIGHT_VIEW::MOVE_BOTH_ARMS - moving right arm to inspect pose while left arm clears to wait_resources");
  }
};

struct StInspectRightViewWaitAck : smacc2::SmaccState<StInspectRightViewWaitAck, StInspectRightView>
{
  using SmaccState::SmaccState;

  void onEntry()
  {
    CpBusinessKeyMapper * keyMapper = nullptr;
    this->requiresComponent(keyMapper);
    if (keyMapper != nullptr)
    {
      keyMapper->request_back_inspection(this->getClassName());
    }

    RCLCPP_INFO(
      getLogger(),
      "WORK::INSPECT::RIGHT_VIEW::WAIT_ACK - both arms are in the inspect posture, attempting external ROS trigger for back-side inspection; press 'n' to continue manually if needed");
  }
};

}  // namespace inspect_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm