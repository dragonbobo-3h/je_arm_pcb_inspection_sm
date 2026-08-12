#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/pick_substates/st_l_pregrasp.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/pick_substates/st_gripper_open.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/pick_substates/st_cartesian_down.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/pick_substates/st_gripper_close.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/pick_substates/st_cartesian_up.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/pick_substates/st_l_retreat.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/pick_substates/st_grasp_check.hpp"
#include "je_arm_pcb_inspection_sm/utils/logging.hpp"

namespace je_arm_pcb_inspection_sm
{

namespace work_substates
{

struct StPick;

namespace pick_substates
{

struct StLPregrasp;
struct StGripperOpen;
struct StCartesianDown;
struct StGripperClose;
struct StCartesianUp;
struct StLRetreat;
struct StGraspCheck;

struct StPickResumeRouter : smacc2::SmaccState<StPickResumeRouter, StPick>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvPickResumeToLPregrasp, StLPregrasp>,
    smacc2::Transition<EvPickResumeToGripperOpen, StGripperOpen>,
    smacc2::Transition<EvPickResumeToCartesianDown, StCartesianDown>,
    smacc2::Transition<EvPickResumeToGripperClose, StGripperClose>,
    smacc2::Transition<EvPickResumeToCartesianUp, StCartesianUp>,
    smacc2::Transition<EvPickResumeToLRetreat, StLRetreat>,
    smacc2::Transition<EvPickResumeToGraspCheck, StGraspCheck>
  > reactions;

  void onEntry()
  {
    std::string substate = sm_data::kPickSubstateLPregrasp;
    this->getGlobalSMData(std::string(sm_data::kPickResumeSubstateId), substate);

    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] TRANSITION PICK::RESUME_ROUTER --> %s",
      log_utils::bjtNowString().c_str(),
      substate.c_str());
    if (substate == sm_data::kPickSubstateLPregrasp)
    {
      this->template postEvent<EvPickResumeToLPregrasp>();
    }
    else if (substate == sm_data::kPickSubstateGripperOpen)
    {
      this->template postEvent<EvPickResumeToGripperOpen>();
    }
    else if (substate == sm_data::kPickSubstateCartesianDown)
    {
      this->template postEvent<EvPickResumeToCartesianDown>();
    }
    else if (substate == sm_data::kPickSubstateGripperClose)
    {
      this->template postEvent<EvPickResumeToGripperClose>();
    }
    else if (substate == sm_data::kPickSubstateCartesianUp)
    {
      this->template postEvent<EvPickResumeToCartesianUp>();
    }
    else if (substate == sm_data::kPickSubstateLRetreat)
    {
      this->template postEvent<EvPickResumeToLRetreat>();
    }
    else if (substate == sm_data::kPickSubstateGraspCheck)
    {
      this->template postEvent<EvPickResumeToGraspCheck>();
    }
    else
    {
      // Default: start pregrasp stage (single pregrasp move state).
      this->template postEvent<EvPickResumeToLPregrasp>();
    }
  }
};

}  // namespace pick_substates

}  // namespace work_substates

}  // namespace je_arm_pcb_inspection_sm
