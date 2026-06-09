#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"

namespace je_arm_pcb_inspection_sm
{

struct StWork;
struct StPause;

namespace work_substates
{

namespace inspect_substates
{
struct StInspectResumeRouter;
}  // namespace inspect_substates

struct StInspect : smacc2::SmaccState<StInspect, StWork, inspect_substates::StInspectResumeRouter>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  void onEntry()
  {
    bool inspectResumeFromPause = false;
    this->getGlobalSMData(std::string(sm_data::kInspectResumeFromPause), inspectResumeFromPause);

    if (!inspectResumeFromPause)
    {
      this->setGlobalSMData(
        std::string(sm_data::kInspectResumeSubstateId),
        std::string(sm_data::kInspectSubstateFrontPose));
    }

    this->setGlobalSMData(std::string(sm_data::kInspectResumeFromPause), false);
    this->setGlobalSMData(
      std::string(sm_data::kWorkResumeSubstateId),
      std::string(sm_data::kWorkSubstateInspect));
    this->setGlobalSMData(
      std::string(sm_data::kResumeStateId),
      std::string(sm_data::kWorkState));
    RCLCPP_INFO(
      getLogger(),
      "WORK::INSPECT onEntry - expanded flow (front inspect -> handover -> back inspect -> handover back)");
  }
};

}  // namespace work_substates

}  // namespace je_arm_pcb_inspection_sm

#include "je_arm_pcb_inspection_sm/states/work_substates/inspect_substates/st_inspect_resume_router.hpp"
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
