#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/components/cp_top_level_flow.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"

namespace je_arm_pcb_inspection_sm
{

struct StWork;
struct StPause;

namespace work_substates
{

namespace pick_substates
{
struct StPickResumeRouter;
}  // namespace pick_substates

struct StInspect;

struct StPick : smacc2::SmaccState<StPick, StWork, pick_substates::StPickResumeRouter>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvPickDone, StInspect>,
    smacc2::Transition<EvPickRetry, StPick>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void onEntry()
  {
    CpTopLevelFlow * flow = nullptr;
    this->requiresComponent(flow);
    flow->beginLoopCycle();
    bool pickResumeFromPause = false;
    this->getGlobalSMData(std::string(sm_data::kPickResumeFromPause), pickResumeFromPause);

    if (!pickResumeFromPause)
    {
      this->setGlobalSMData(
        std::string(sm_data::kPickResumeSubstateId),
        std::string(sm_data::kPickSubstateLPregrasp));
    }

    this->setGlobalSMData(std::string(sm_data::kPickResumeFromPause), false);
    this->setGlobalSMData(
      std::string(sm_data::kWorkResumeSubstateId),
      std::string(sm_data::kWorkSubstatePick));
    RCLCPP_INFO(
      getLogger(),
      "WORK::PICK onEntry - nested flow started (L_PREGRASP->GRIPPER_OPEN->CARTESIAN_DOWN->GRIPPER_CLOSE->CARTESIAN_UP->L_RETREAT)");
  }
};

}  // namespace work_substates

}  // namespace je_arm_pcb_inspection_sm

#include "je_arm_pcb_inspection_sm/states/work_substates/pick_substates/st_pick_resume_router.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/pick_substates/st_l_pregrasp.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/pick_substates/st_gripper_open.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/pick_substates/st_cartesian_down.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/pick_substates/st_gripper_close.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/pick_substates/st_cartesian_up.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/pick_substates/st_l_retreat.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/pick_substates/st_grasp_check.hpp"
