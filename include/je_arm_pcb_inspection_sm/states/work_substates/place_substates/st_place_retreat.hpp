#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/states/work_substates/place_substates/cb_move_place_target_pose.hpp"

namespace je_arm_pcb_inspection_sm
{

struct StPause;
struct StDelay;

namespace work_substates
{

struct StPlace;

namespace place_substates
{

struct StPlaceGripperClose;

struct StPlaceCartesianUp : smacc2::SmaccState<StPlaceCartesianUp, StPlace>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<
      smacc2::EvCbSuccess<CbMovePlaceTargetPose, OrArm>,
      StDelay>,
    smacc2::Transition<
      smacc2::EvCbFailure<CbMovePlaceTargetPose, OrArm>,
      StPause>,
    smacc2::Transition<EvPauseRequested, StPause>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal_runtime<OrArm, CbMovePlaceTargetPose>(
      [](CbMovePlaceTargetPose & bh, StPlaceCartesianUp & state)
      {
        bh.useYamlOffset_ = true;
      });
  }

  void onEntry()
  {
    bool loopMode = false;
    bool returningToPick = false;
    this->getGlobalSMData(std::string(sm_data::kLoopMode), loopMode);
    this->getGlobalSMData(std::string(sm_data::kReturningToPick), returningToPick);
    this->setGlobalSMData(
      std::string(sm_data::kPlaceResumeSubstateId),
      std::string(sm_data::kPlaceSubstateCartesianUp));
    this->setGlobalSMData(
      std::string(sm_data::kPlaceDelayNextSubstateId),
      std::string(loopMode && !returningToPick ?
        sm_data::kPlaceSubstateReturnToPick : sm_data::kPlaceSubstateGripperClose));
    this->setGlobalSMData(
      std::string(sm_data::kResumeStateId),
      std::string(sm_data::kWorkState));
    this->setGlobalSMData(
      std::string(sm_data::kWorkResumeSubstateId),
      std::string(sm_data::kWorkSubstatePlace));
    this->setGlobalSMData(
      std::string(sm_data::kWorkDelayNextSubstateId),
      std::string(sm_data::kWorkSubstatePlace));
    RCLCPP_INFO(getLogger(), "WORK::PLACE::CARTESIAN_UP - moving to place pose using place_offset.yaml");
  }
};

}  // namespace place_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm
