#pragma once

#include <smacc2/smacc.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"

namespace je_arm_pcb_inspection_sm
{
namespace work_substates
{

struct StPlace;

namespace place_substates
{

struct StPlaceLPregrasp;
struct StPlaceCartesianDown;
struct StPlaceGripperOpen;
struct StPlaceCartesianUp;
struct StPlaceGripperClose;
struct StPlaceReturnToPick;

struct StPlaceResumeRouter : smacc2::SmaccState<StPlaceResumeRouter, StPlace>
{
  using SmaccState::SmaccState;

  typedef boost::mpl::list<
    smacc2::Transition<EvPlaceResumeToPre, StPlaceLPregrasp>,
    smacc2::Transition<EvPlaceResumeToMove, StPlaceCartesianDown>,
    smacc2::Transition<EvPlaceResumeToRelease, StPlaceGripperOpen>,
    smacc2::Transition<EvPlaceResumeToRetreat, StPlaceCartesianUp>,
    smacc2::Transition<EvPlaceResumeToClose, StPlaceGripperClose>,
    smacc2::Transition<EvPlaceResumeToReturnToPick, StPlaceReturnToPick>
  > reactions;

  void onEntry()
  {
    std::string substate = sm_data::kPlaceSubstateLPregrasp;
    this->getGlobalSMData(std::string(sm_data::kPlaceResumeSubstateId), substate);

    if (substate == sm_data::kPlaceSubstateCartesianDown)
    {
      this->template postEvent<EvPlaceResumeToMove>();
    }
    else if (substate == sm_data::kPlaceSubstateGripperOpen)
    {
      this->template postEvent<EvPlaceResumeToRelease>();
    }
    else if (substate == sm_data::kPlaceSubstateCartesianUp)
    {
      this->template postEvent<EvPlaceResumeToRetreat>();
    }
    else if (substate == sm_data::kPlaceSubstateGripperClose)
    {
      this->template postEvent<EvPlaceResumeToClose>();
    }
    else if (substate == sm_data::kPlaceSubstateReturnToPick)
    {
      this->template postEvent<EvPlaceResumeToReturnToPick>();
    }
    else
    {
      this->template postEvent<EvPlaceResumeToPre>();
    }
  }
};

}  // namespace place_substates
}  // namespace work_substates
}  // namespace je_arm_pcb_inspection_sm
