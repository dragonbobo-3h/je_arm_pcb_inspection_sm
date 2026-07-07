#pragma once

#include <cmath>
#include <common/msg/pcb_detection.hpp>
#include <common/msg/place_slot.hpp>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc.hpp>

#include "je_arm_pcb_inspection_sm/sm_data.hpp"

namespace je_arm_pcb_inspection_sm
{

class CpTopLevelFlow : public smacc2::ISmaccComponent
{
public:
  CpTopLevelFlow() = default;
  virtual ~CpTopLevelFlow() = default;

  template <typename TOrthogonal, typename TClient>
  void onComponentInitialization()
  {
    auto node = this->getNode();
    loadTopicNames(node);

    pcbDetectionSub_ = node->create_subscription<common::msg::PcbDetection>(
      pcbDetectionTopic_,
      rclcpp::QoS(10),
      [this](const common::msg::PcbDetection::SharedPtr msg)
      {
        std::scoped_lock<std::mutex> lock(dataMutex_);
        if (msg->present && isPoseValid(msg->pose.pose))
        {
          latestPcbPose_ = msg->pose;
          hasValidPcbPose_ = true;
          RCLCPP_INFO(
            getLogger(),
            "[CpTopLevelFlow] Received PICK target from topic: frame=%s pos=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
            latestPcbPose_.header.frame_id.c_str(),
            latestPcbPose_.pose.position.x,
            latestPcbPose_.pose.position.y,
            latestPcbPose_.pose.position.z,
            latestPcbPose_.pose.orientation.x,
            latestPcbPose_.pose.orientation.y,
            latestPcbPose_.pose.orientation.z,
            latestPcbPose_.pose.orientation.w);
        }
        else
        {
          hasValidPcbPose_ = false;
        }
      });

    placeSlotSub_ = node->create_subscription<common::msg::PlaceSlot>(
      placeSlotTopic_,
      rclcpp::QoS(10),
      [this](const common::msg::PlaceSlot::SharedPtr msg)
      {
        std::scoped_lock<std::mutex> lock(dataMutex_);
        hasPlaceSlotUpdate_ = true;
        if (msg->free && isPoseValid(msg->pose.pose))
        {
          latestPlacePose_ = msg->pose;
          hasFreePlaceSlot_ = true;
          RCLCPP_INFO(
            getLogger(),
            "[CpTopLevelFlow] Received PLACE target from topic: frame=%s pos=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
            latestPlacePose_.header.frame_id.c_str(),
            latestPlacePose_.pose.position.x,
            latestPlacePose_.pose.position.y,
            latestPlacePose_.pose.position.z,
            latestPlacePose_.pose.orientation.x,
            latestPlacePose_.pose.orientation.y,
            latestPlacePose_.pose.orientation.z,
            latestPlacePose_.pose.orientation.w);
        }
        else
        {
          hasFreePlaceSlot_ = false;
        }
      });

    RCLCPP_INFO(
      getLogger(),
      "[CpTopLevelFlow] Subscribed: pcb_detection='%s' (present+pose)",
      pcbDetectionTopic_.c_str());
    RCLCPP_INFO(
      getLogger(),
      "[CpTopLevelFlow] Subscribed: place_slot='%s' (free+pose)",
      placeSlotTopic_.c_str());
  }

  void setResumeTarget(const char * target)
  {
    this->getStateMachine()->setGlobalSMData(std::string(sm_data::kResumeStateId), std::string(target));
  }

  bool isWorkReady()
  {
    syncTopicDataToBlackboard();

    std::string workSubstate = sm_data::kWorkSubstatePick;
    this->getStateMachine()->getGlobalSMData(std::string(sm_data::kWorkResumeSubstateId), workSubstate);

    if (workSubstate == sm_data::kWorkSubstatePlace)
    {
      return hasPlaceTargetReady();
    }

    return hasPickupWorkReady();
  }

  bool hasPickupWorkReady()
  {

    const bool pcbPresent = getBoolData(sm_data::kPcbPresent, true);
    const bool leftSlotFree = getBoolData(sm_data::kLeftSlotFree, false);
    const bool rightSlotFree = getBoolData(sm_data::kRightSlotFree, true);
    return pcbPresent && (leftSlotFree || rightSlotFree);
  }

  bool hasPlaceTargetReady()
  {
    const bool leftSlotFree = getBoolData(sm_data::kLeftSlotFree, false);
    const bool rightSlotFree = getBoolData(sm_data::kRightSlotFree, false);
    return leftSlotFree || rightSlotFree;
  }

  void setPcbPresent(bool value)
  {
    this->getStateMachine()->setGlobalSMData(std::string(sm_data::kPcbPresent), value);
  }

  double waitTimeoutSeconds()
  {
    return getDoubleData(sm_data::kWaitTimeoutSec, 10.0);
  }

  double workCycleSeconds()
  {
    return getDoubleData(sm_data::kWorkCycleSec, 2.0);
  }

  double backHomeSeconds()
  {
    return getDoubleData(sm_data::kBackHomeSec, 1.5);
  }

private:
  void loadTopicNames(const rclcpp::Node::SharedPtr & node)
  {
    if (!node->has_parameter("pcb_detection_topic"))
    {
      node->declare_parameter<std::string>("pcb_detection_topic", "/pcb_detection");
    }

    if (!node->has_parameter("place_slot_topic"))
    {
      node->declare_parameter<std::string>("place_slot_topic", "/vision/place_slot_detection");
    }

    pcbDetectionTopic_ = node->get_parameter("pcb_detection_topic").as_string();
    placeSlotTopic_ = node->get_parameter("place_slot_topic").as_string();
  }

  void syncTopicDataToBlackboard()
  {
    std::scoped_lock<std::mutex> lock(dataMutex_);

    this->getStateMachine()->setGlobalSMData(
      std::string(sm_data::kPcbPresent), hasValidPcbPose_);

    if (hasValidPcbPose_)
    {
      this->getStateMachine()->setGlobalSMData(
        std::string(sm_data::kPcbTargetFrameId), latestPcbPose_.header.frame_id);
      this->getStateMachine()->setGlobalSMData(
        std::string(sm_data::kPcbTargetX), latestPcbPose_.pose.position.x);
      this->getStateMachine()->setGlobalSMData(
        std::string(sm_data::kPcbTargetY), latestPcbPose_.pose.position.y);
      this->getStateMachine()->setGlobalSMData(
        std::string(sm_data::kPcbTargetZ), latestPcbPose_.pose.position.z);
      this->getStateMachine()->setGlobalSMData(
        std::string(sm_data::kPcbTargetQx), latestPcbPose_.pose.orientation.x);
      this->getStateMachine()->setGlobalSMData(
        std::string(sm_data::kPcbTargetQy), latestPcbPose_.pose.orientation.y);
      this->getStateMachine()->setGlobalSMData(
        std::string(sm_data::kPcbTargetQz), latestPcbPose_.pose.orientation.z);
      this->getStateMachine()->setGlobalSMData(
        std::string(sm_data::kPcbTargetQw), latestPcbPose_.pose.orientation.w);
    }

    if (hasPlaceSlotUpdate_)
    {
      this->getStateMachine()->setGlobalSMData(
        std::string(sm_data::kLeftSlotFree), hasFreePlaceSlot_);
      this->getStateMachine()->setGlobalSMData(
        std::string(sm_data::kRightSlotFree), false);

      if (hasFreePlaceSlot_)
      {
        this->getStateMachine()->setGlobalSMData(
          std::string(sm_data::kPlaceTargetFrameId), latestPlacePose_.header.frame_id);
        this->getStateMachine()->setGlobalSMData(
          std::string(sm_data::kPlaceTargetX), latestPlacePose_.pose.position.x);
        this->getStateMachine()->setGlobalSMData(
          std::string(sm_data::kPlaceTargetY), latestPlacePose_.pose.position.y);
        this->getStateMachine()->setGlobalSMData(
          std::string(sm_data::kPlaceTargetZ), latestPlacePose_.pose.position.z);
        this->getStateMachine()->setGlobalSMData(
          std::string(sm_data::kPlaceTargetQx), latestPlacePose_.pose.orientation.x);
        this->getStateMachine()->setGlobalSMData(
          std::string(sm_data::kPlaceTargetQy), latestPlacePose_.pose.orientation.y);
        this->getStateMachine()->setGlobalSMData(
          std::string(sm_data::kPlaceTargetQz), latestPlacePose_.pose.orientation.z);
        this->getStateMachine()->setGlobalSMData(
          std::string(sm_data::kPlaceTargetQw), latestPlacePose_.pose.orientation.w);
      }
    }
  }

  bool isPoseValid(const geometry_msgs::msg::Pose & pose) const
  {
    const bool positionFinite =
      std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
      std::isfinite(pose.position.z);

    const bool orientationFinite =
      std::isfinite(pose.orientation.x) && std::isfinite(pose.orientation.y) &&
      std::isfinite(pose.orientation.z) && std::isfinite(pose.orientation.w);

    const double qnorm2 =
      pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
      pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w;

    return positionFinite && orientationFinite && qnorm2 > 1e-12;
  }

  bool getBoolData(const char * key, bool fallback)
  {
    bool value = fallback;
    this->getStateMachine()->getGlobalSMData(std::string(key), value);
    return value;
  }

  double getDoubleData(const char * key, double fallback)
  {
    double value = fallback;
    this->getStateMachine()->getGlobalSMData(std::string(key), value);
    return value;
  }

  std::mutex dataMutex_;
  bool hasValidPcbPose_{false};
  geometry_msgs::msg::PoseStamped latestPcbPose_;
  bool hasPlaceSlotUpdate_{false};
  bool hasFreePlaceSlot_{false};
  geometry_msgs::msg::PoseStamped latestPlacePose_;

  std::string pcbDetectionTopic_{"/pcb_detection"};
  std::string placeSlotTopic_{"/vision/place_slot_detection"};

  rclcpp::Subscription<common::msg::PcbDetection>::SharedPtr pcbDetectionSub_;
  rclcpp::Subscription<common::msg::PlaceSlot>::SharedPtr placeSlotSub_;
};

}  // namespace je_arm_pcb_inspection_sm