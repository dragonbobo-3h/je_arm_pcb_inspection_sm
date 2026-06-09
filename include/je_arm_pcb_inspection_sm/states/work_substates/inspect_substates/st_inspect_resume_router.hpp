#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/inspect_substates/st_inspect_front_pose.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/inspect_substates/st_inspect_align_for_right_handover.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/inspect_substates/st_inspect_right_both_grippers_close.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/inspect_substates/st_inspect_right_gripper_open_receive.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/inspect_substates/st_inspect_right_approach.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/inspect_substates/st_inspect_right_gripper_close.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/inspect_substates/st_inspect_left_gripper_open.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/inspect_substates/st_inspect_right_retreat.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/inspect_substates/st_inspect_right_view.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/inspect_substates/st_inspect_align_for_left_handover.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/inspect_substates/st_inspect_left_both_grippers_close.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/inspect_substates/st_inspect_left_gripper_open_receive.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/inspect_substates/st_inspect_left_approach.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/inspect_substates/st_inspect_left_gripper_close.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/inspect_substates/st_inspect_right_gripper_open.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/inspect_substates/st_inspect_left_retreat.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/inspect_substates/st_inspect_return_wait_pose.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/inspect_substates/st_inspect_wait_place_result.hpp"
#include "je_arm_pcb_inspection_sm/utils/logging.hpp"

namespace je_arm_pcb_inspection_sm
{
namespace work_substates
{

struct StInspect;

namespace inspect_substates
{

struct StInspectResumeRouter : smacc2::SmaccState<StInspectResumeRouter, StInspect>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvInspectResumeToFrontPose, StInspectFrontPose>,
    smacc2::Transition<EvInspectResumeToAlignForRightHandover, StInspectAlignForRightHandover>,
    smacc2::Transition<EvInspectResumeToRightBothGrippersClose, StInspectRightBothGrippersClose>,
    smacc2::Transition<EvInspectResumeToRightGripperOpenReceive, StInspectRightGripperOpenReceive>,
    smacc2::Transition<EvInspectResumeToRightApproach, StInspectRightApproach>,
    smacc2::Transition<EvInspectResumeToRightGripperClose, StInspectRightGripperClose>,
    smacc2::Transition<EvInspectResumeToLeftGripperOpen, StInspectLeftGripperOpen>,
    smacc2::Transition<EvInspectResumeToRightRetreat, StInspectRightRetreat>,
    smacc2::Transition<EvInspectResumeToRightView, StInspectRightView>,
    smacc2::Transition<EvInspectResumeToAlignForLeftHandover, StInspectAlignForLeftHandover>,
    smacc2::Transition<EvInspectResumeToLeftBothGrippersClose, StInspectLeftBothGrippersClose>,
    smacc2::Transition<EvInspectResumeToLeftGripperOpenReceive, StInspectLeftGripperOpenReceive>,
    smacc2::Transition<EvInspectResumeToLeftApproach, StInspectLeftApproach>,
    smacc2::Transition<EvInspectResumeToLeftGripperClose, StInspectLeftGripperClose>,
    smacc2::Transition<EvInspectResumeToRightGripperOpen, StInspectRightGripperOpen>,
    smacc2::Transition<EvInspectResumeToLeftRetreat, StInspectLeftRetreat>,
    smacc2::Transition<EvInspectResumeToReturnWaitPose, StInspectReturnWaitPose>,
    smacc2::Transition<EvInspectResumeToWaitPlaceResult, StInspectWaitPlaceResult>
  > reactions;

  void onEntry()
  {
    std::string substate = sm_data::kInspectSubstateFrontPose;
    this->getGlobalSMData(std::string(sm_data::kInspectResumeSubstateId), substate);

    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] TRANSITION INSPECT::RESUME_ROUTER --> %s",
      log_utils::bjtNowString().c_str(),
      substate.c_str());

    if (substate == sm_data::kInspectSubstateAlignForRightHandover)
    {
      this->template postEvent<EvInspectResumeToAlignForRightHandover>();
    }
    else if (substate == sm_data::kInspectSubstateRightBothGrippersClose)
    {
      this->template postEvent<EvInspectResumeToRightBothGrippersClose>();
    }
    else if (substate == sm_data::kInspectSubstateRightGripperOpenReceive)
    {
      this->template postEvent<EvInspectResumeToRightGripperOpenReceive>();
    }
    else if (substate == sm_data::kInspectSubstateRightApproach)
    {
      this->template postEvent<EvInspectResumeToRightApproach>();
    }
    else if (substate == sm_data::kInspectSubstateRightGripperClose)
    {
      this->template postEvent<EvInspectResumeToRightGripperClose>();
    }
    else if (substate == sm_data::kInspectSubstateLeftGripperOpen)
    {
      this->template postEvent<EvInspectResumeToLeftGripperOpen>();
    }
    else if (substate == sm_data::kInspectSubstateRightRetreat)
    {
      this->template postEvent<EvInspectResumeToRightRetreat>();
    }
    else if (substate == sm_data::kInspectSubstateRightView)
    {
      this->template postEvent<EvInspectResumeToRightView>();
    }
    else if (substate == sm_data::kInspectSubstateAlignForLeftHandover)
    {
      this->template postEvent<EvInspectResumeToAlignForLeftHandover>();
    }
    else if (substate == sm_data::kInspectSubstateLeftBothGrippersClose)
    {
      this->template postEvent<EvInspectResumeToLeftBothGrippersClose>();
    }
    else if (substate == sm_data::kInspectSubstateLeftGripperOpenReceive)
    {
      this->template postEvent<EvInspectResumeToLeftGripperOpenReceive>();
    }
    else if (substate == sm_data::kInspectSubstateLeftApproach)
    {
      this->template postEvent<EvInspectResumeToLeftApproach>();
    }
    else if (substate == sm_data::kInspectSubstateLeftGripperClose)
    {
      this->template postEvent<EvInspectResumeToLeftGripperClose>();
    }
    else if (substate == sm_data::kInspectSubstateRightGripperOpen)
    {
      this->template postEvent<EvInspectResumeToRightGripperOpen>();
    }
    else if (substate == sm_data::kInspectSubstateLeftRetreat)
    {
      this->template postEvent<EvInspectResumeToLeftRetreat>();
    }
    else if (substate == sm_data::kInspectSubstateReturnWaitPose)
    {
      this->template postEvent<EvInspectResumeToReturnWaitPose>();
    }
    else if (substate == sm_data::kInspectSubstateWaitPlaceResult)
    {
      this->template postEvent<EvInspectResumeToWaitPlaceResult>();
    }
    else
    {
      this->template postEvent<EvInspectResumeToFrontPose>();
    }
  }
};

}  // namespace inspect_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm