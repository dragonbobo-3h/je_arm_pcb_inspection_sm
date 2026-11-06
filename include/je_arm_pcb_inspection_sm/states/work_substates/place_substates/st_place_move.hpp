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

struct StPlaceGripperOpen;

struct StPlaceCartesianDown : smacc2::SmaccState<StPlaceCartesianDown, StPlace>
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
      [](CbMovePlaceTargetPose & bh, StPlaceCartesianDown & state)
      {
        bh.useYamlOffset_ = false;
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
      std::string(sm_data::kPlaceSubstateCartesianDown));
    this->setGlobalSMData(
      std::string(sm_data::kPlaceDelayNextSubstateId),
      std::string(loopMode && !returningToPick ?
        sm_data::kPlaceSubstateCartesianUp : sm_data::kPlaceSubstateGripperOpen));
    this->setGlobalSMData(
      std::string(sm_data::kResumeStateId),
      std::string(sm_data::kWorkState));
    this->setGlobalSMData(
      std::string(sm_data::kWorkResumeSubstateId),
      std::string(sm_data::kWorkSubstatePlace));
    this->setGlobalSMData(
      std::string(sm_data::kWorkDelayNextSubstateId),
      std::string(sm_data::kWorkSubstatePlace));
    RCLCPP_INFO(getLogger(), "WORK::PLACE::CARTESIAN_DOWN - moving to place pose");
  }
};

}  // namespace place_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm
