#pragma once

#include <chrono>
#include <ctime>
#include <iomanip>
#include <cl_moveit2z/cl_moveit2z.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>

#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"

namespace je_arm_pcb_inspection_sm::log_utils
{
inline rclcpp::Logger bizLogger()
{
  return rclcpp::get_logger("je_arm_pcb_inspection_sm.biz");
}

inline std::string bjtNowString()
{
  using namespace std::chrono;
  const auto nowBjt = system_clock::now() + hours(8);
  const auto ms = duration_cast<milliseconds>(nowBjt.time_since_epoch()) % 1000;

  const std::time_t t = system_clock::to_time_t(nowBjt);
  std::tm tm{};
  gmtime_r(&t, &tm);

  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S")
      << '.' << std::setw(3) << std::setfill('0') << ms.count()
      << " BJT";
  return oss.str();
}

inline std::string formatArmSnapshot(smacc2::ISmaccStateMachine & sm)
{
  auto * orthogonal = sm.getOrthogonal<je_arm_pcb_inspection_sm::OrLeftArm>();
  if (orthogonal == nullptr)
  {
    return "arm_state=unavailable(or_left_arm=null)";
  }

  cl_moveit2z::ClMoveit2z * moveitClient = nullptr;
  if (!orthogonal->requiresClient(moveitClient) || moveitClient == nullptr ||
    moveitClient->moveGroupClientInterface == nullptr)
  {
    return "arm_state=unavailable(moveit_client=null)";
  }

  auto & mgi = *moveitClient->moveGroupClientInterface;
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4);

  try
  {
    const auto jointNames = mgi.getJointNames();
    const auto jointValues = mgi.getCurrentJointValues();

    oss << "joints={";
    for (size_t i = 0; i < jointNames.size() && i < jointValues.size(); ++i)
    {
      if (i > 0) oss << ", ";
      oss << jointNames[i] << ':' << jointValues[i];
    }
    oss << '}';

    const auto tipLink = je_arm_pcb_inspection_sm::getLeftTipLink(sm.getNode());
    const auto poseStamped = mgi.getCurrentPose(tipLink);
    const auto & p = poseStamped.pose.position;
    const auto & q = poseStamped.pose.orientation;
    oss << " | ee(link=" << tipLink << ", frame=" << poseStamped.header.frame_id
        << ", pos=[" << p.x << ", " << p.y << ", " << p.z << ']'
        << ", quat=[" << q.x << ", " << q.y << ", " << q.z << ", " << q.w << "])";
  }
  catch (const std::exception & e)
  {
    return std::string("arm_state=exception(") + e.what() + ')';
  }

  return oss.str();
}

inline std::string formatArmSnapshot(smacc2::ISmaccStateMachine * sm)
{
  if (sm == nullptr)
  {
    return "arm_state=unavailable(sm=null)";
  }

  return formatArmSnapshot(*sm);
}

inline void logArmSnapshot(smacc2::ISmaccStateMachine & sm, const std::string & prefix)
{
  RCLCPP_INFO(
    bizLogger(),
    "[%s] %s | %s",
    bjtNowString().c_str(),
    prefix.c_str(),
    formatArmSnapshot(sm).c_str());
}

inline void logArmSnapshot(smacc2::ISmaccStateMachine * sm, const std::string & prefix)
{
  RCLCPP_INFO(
    bizLogger(),
    "[%s] %s | %s",
    bjtNowString().c_str(),
    prefix.c_str(),
    formatArmSnapshot(sm).c_str());
}
}  // namespace je_arm_pcb_inspection_sm::log_utils
