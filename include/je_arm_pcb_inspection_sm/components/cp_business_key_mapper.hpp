#pragma once

#include <cctype>
#include <chrono>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <common/msg/pcb_detection.hpp>
#include <common/msg/place_slot.hpp>
#include <common/srv/inspect_capture.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc.hpp>
#include <std_msgs/msg/bool.hpp>
#include <yaml-cpp/yaml.h>

#include <cl_keyboard/components/cp_keyboard_listener_1.hpp>
#include <cl_moveit2z/cl_moveit2z.hpp>
#include <cl_moveit2z/components/cp_trajectory_executor.hpp>

#include "je_arm_pcb_inspection_sm/events.hpp"
#include "je_arm_pcb_inspection_sm/orthogonals/or_arm.hpp"
#include "je_arm_pcb_inspection_sm/sm_data.hpp"
#include "je_arm_pcb_inspection_sm/utils/logging.hpp"

namespace je_arm_pcb_inspection_sm
{

class CpBusinessKeyMapper : public smacc2::ISmaccComponent
{
public:
  CpBusinessKeyMapper() = default;
  virtual ~CpBusinessKeyMapper() = default;

  template <typename TOrthogonal, typename TClient>
  void onComponentInitialization()
  {
    RCLCPP_WARN(getLogger(), "CpBusinessKeyMapper::onComponentInitialization - STARTING");
    cl_keyboard::components::CpKeyboardListener1 * keyboardListener;
    this->requiresComponent(keyboardListener);
    RCLCPP_WARN(getLogger(), "CpBusinessKeyMapper - got keyboard listener, registering callback");
    keyboardListener->OnKeyPress(&CpBusinessKeyMapper::onKeyPress, this);

    auto node = this->getNode();
    loadTopicNames(node);
    pcbDetectionPub_ =
      node->create_publisher<common::msg::PcbDetection>(
        pcbDetectionTopic_, rclcpp::QoS(10));
    placeSlotPub_ =
      node->create_publisher<common::msg::PlaceSlot>(
        placeSlotTopic_, rclcpp::QoS(10));
    inspectDonePub_ =
      node->create_publisher<std_msgs::msg::Bool>(
        inspectDoneTopic_, rclcpp::QoS(10));
    inspectDoneSub_ =
      node->create_subscription<std_msgs::msg::Bool>(
        inspectDoneTopic_,
        rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
          if (msg != nullptr && msg->data)
          {
            this->handleInspectDoneSignal("topic");
          }
        });
    inspectFrontTriggerClient_ = node->create_client<common::srv::InspectCapture>(inspectFrontTriggerService_);
    inspectBackTriggerClient_ = node->create_client<common::srv::InspectCapture>(inspectBackTriggerService_);

    RCLCPP_INFO(log_utils::bizLogger(), "[%s] KEY_MAPPER ready", log_utils::bjtNowString().c_str());
    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] KEY_MAPPER subscribed: inspect_done='%s' inspect_front_trigger='%s' inspect_back_trigger='%s'",
      log_utils::bjtNowString().c_str(),
      inspectDoneTopic_.c_str(),
      inspectFrontTriggerService_.c_str(),
      inspectBackTriggerService_.c_str());
  }

  void request_front_inspection(const std::string & waitStateName)
  {
    request_inspection(waitStateName, inspectFrontTriggerClient_, inspectFrontTriggerService_, "front");
  }

  void request_back_inspection(const std::string & waitStateName)
  {
    request_inspection(waitStateName, inspectBackTriggerClient_, inspectBackTriggerService_, "back");
  }

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

    if (!node->has_parameter("inspect_done_topic"))
    {
      node->declare_parameter<std::string>("inspect_done_topic", "/vision/inspect_done");
    }

    if (!node->has_parameter("inspect_front_trigger_service"))
    {
      node->declare_parameter<std::string>("inspect_front_trigger_service", "/vision/inspect_front");
    }

    if (!node->has_parameter("inspect_back_trigger_service"))
    {
      node->declare_parameter<std::string>("inspect_back_trigger_service", "/vision/inspect_back");
    }

    pcbDetectionTopic_ = node->get_parameter("pcb_detection_topic").as_string();
    placeSlotTopic_ = node->get_parameter("place_slot_topic").as_string();
    inspectDoneTopic_ = node->get_parameter("inspect_done_topic").as_string();
    inspectFrontTriggerService_ = node->get_parameter("inspect_front_trigger_service").as_string();
    inspectBackTriggerService_ = node->get_parameter("inspect_back_trigger_service").as_string();
  }

  void onKeyPress(char key)
  {
    auto * currentState = this->getStateMachine()->getCurrentState();
    if (currentState == nullptr)
    {
      RCLCPP_WARN(getLogger(), "CpBusinessKeyMapper::onKeyPress - currentState is nullptr");
      return;
    }

    const std::string stateName = currentState->getClassName();
    const char normalizedKey = static_cast<char>(std::tolower(static_cast<unsigned char>(key)));
    const bool isInspectSubstate = stateName.find("StInspect") != std::string::npos;
    const bool isWorkPhaseState =
      stateName.find("StWork") != std::string::npos ||
      stateName.find("StPick") != std::string::npos ||
      stateName.find("StLPregrasp") != std::string::npos ||
      stateName.find("StGripperOpen") != std::string::npos ||
      stateName.find("StCartesianDown") != std::string::npos ||
      stateName.find("StGripperClose") != std::string::npos ||
      stateName.find("StCartesianUp") != std::string::npos ||
      stateName.find("StLRetreat") != std::string::npos ||
      isInspectSubstate ||
      stateName.find("StPlace") != std::string::npos;

    RCLCPP_INFO(
      getLogger(),
      "CpBusinessKeyMapper::onKeyPress - key='%c' normalized='%c' state=%s",
      key,
      normalizedKey,
      stateName.c_str());

    auto setPauseReasonFromState = [&]() {
      this->getStateMachine()->setGlobalSMData(
        std::string(sm_data::kPauseReason),
        std::string("[") + log_utils::bjtNowString() + "] 手动暂停: key='p', state=" + stateName);
    };

    auto tryPostPauseRequested = [&](bool setReason) {
      if (normalizedKey != 'p')
      {
        return false;
      }

      if (isPauseDebounced())
      {
        return true;
      }

      if (setReason)
      {
        setPauseReasonFromState();
      }

      this->postEvent<EvPauseRequested>();
      return true;
    };

    if (stateName.find("StIdle") != std::string::npos)
    {
      RCLCPP_WARN(getLogger(), "  -> State matched: StIdle");
      if (normalizedKey == 's')
      {
        RCLCPP_WARN(getLogger(), "  -> Key matched: 's' or 'S', posting EvStartWork");
        this->postEvent<EvStartWork>();
      }
      else if (normalizedKey == 't')
      {
        RCLCPP_WARN(getLogger(), "  -> Key matched: 't' or 'T', posting EvTestGripper");
        this->postEvent<EvTestGripper>();
      }
      else
      {
        RCLCPP_WARN(getLogger(), "  -> Key not matched: got '%c', expected 's' or 't'", key);
      }
      return;
    }

    if (stateName.find("StWaitResources") != std::string::npos)
    {
      if (normalizedKey == 'w')
      {
        publishPcbDetection(true);
      }
      else if (normalizedKey == 'u')
      {
        publishPcbDetection(false);
      }
      else if (normalizedKey == 'h')
      {
        publishFreePlaceSlotFromYaml();
      }
      else if (normalizedKey == 'p')
      {
        (void)tryPostPauseRequested(true);
      }
      else if (normalizedKey == 'i')
      {
        this->postEvent<EvBackToIdleRequested>();
      }
      return;
    }

    if (stateName.find("StActivate") != std::string::npos)
    {
      if (normalizedKey == 'p')
      {
        cancelTrajectoryExecutorIfAvailable();

        (void)tryPostPauseRequested(true);
      }
      return;
    }

    if (isWorkPhaseState)
    {
      if (stateName.find("StLPregrasp") != std::string::npos && normalizedKey == 'n')
      {
        this->postEvent<EvAtPregrasp>();
      }
      else if (stateName.find("StGripperOpen") != std::string::npos && normalizedKey == 'n')
      {
        this->postEvent<EvGripperOpened>();
      }
      else if (stateName.find("StCartesianDown") != std::string::npos && normalizedKey == 'n')
      {
        this->postEvent<EvAtGraspDepth>();
      }
      else if (stateName.find("StGripperClose") != std::string::npos && normalizedKey == 'n')
      {
        this->postEvent<EvGripperClosed>();
      }
      else if (stateName.find("StInspectRightGripperClose") != std::string::npos && normalizedKey == 'n')
      {
        this->postEvent<EvGripperClosed>();
      }
      else if (stateName.find("StInspectLeftGripperClose") != std::string::npos && normalizedKey == 'n')
      {
        this->postEvent<EvGripperClosed>();
      }
      else if (stateName.find("StCartesianUp") != std::string::npos && normalizedKey == 'n')
      {
        this->postEvent<EvLifted>();
      }
      else if (stateName.find("StInspectLeftGripperOpen") != std::string::npos && normalizedKey == 'n')
      {
        this->postEvent<EvGripperOpened>();
      }
      else if (stateName.find("StInspectRightGripperOpen") != std::string::npos && normalizedKey == 'n')
      {
        this->postEvent<EvGripperOpened>();
      }
      else if (stateName.find("StInspectRightViewWaitAck") != std::string::npos && normalizedKey == 'n')
      {
        publishInspectDone();
      }
      else if (stateName.find("StInspectFrontPoseWaitAck") != std::string::npos && normalizedKey == 'n')
      {
        publishInspectDone();
      }
      else if (stateName.find("StInspectRightApproach") != std::string::npos && normalizedKey == 'n')
      {
        this->postEvent<EvInspectStepAck>();
      }
      else if (stateName.find("StInspectLeftApproach") != std::string::npos && normalizedKey == 'n')
      {
        this->postEvent<EvInspectStepAck>();
      }
      else if (stateName.find("StInspectRightRetreat") != std::string::npos && normalizedKey == 'n')
      {
        this->postEvent<EvInspectStepAck>();
      }
      else if (stateName.find("StInspectLeftRetreat") != std::string::npos && normalizedKey == 'n')
      {
        this->postEvent<EvInspectStepAck>();
      }
      else if (stateName.find("StLRetreat") != std::string::npos && normalizedKey == 'n')
      {
        this->postEvent<EvPickDone>();
      }
      else if (stateName.find("StPick") != std::string::npos && normalizedKey == 'n')
      {
        this->postEvent<EvAtPregrasp>();
      }
      else if (stateName.find("StPlaceGripperOpen") != std::string::npos && normalizedKey == 'n')
      {
        this->postEvent<EvPlaceReleased>();
      }
      else if (stateName.find("StPlaceGripperClose") != std::string::npos && normalizedKey == 'n')
      {
        this->postEvent<EvPlaceDone>();
      }
      else if (normalizedKey == 'p')
      {
        cancelTrajectoryExecutorIfAvailable();
        (void)tryPostPauseRequested(false);
      }
      else if (normalizedKey == 'u')
      {
        publishPcbDetection(false);
      }
      else if (normalizedKey == 'h')
      {
        publishFreePlaceSlotFromYaml();
      }
      return;
    }

    if (stateName.find("StBack") != std::string::npos)
    {
      if (normalizedKey == 'p')
      {
        cancelTrajectoryExecutorIfAvailable();
      }
      else if (normalizedKey == 'u')
      {
        publishPcbDetection(false);
        return;
      }
      else if (normalizedKey == 'h')
      {
        publishFreePlaceSlotFromYaml();
        return;
      }

      (void)tryPostPauseRequested(true);
      return;
    }

    if (stateName.find("StDelay") != std::string::npos)
    {
      if (normalizedKey == 'p' && !isPauseDebounced())
      {
        this->getStateMachine()->setGlobalSMData(
          std::string(sm_data::kPauseReason),
          std::string("[") + log_utils::bjtNowString() + "] 手动暂停: key='p', state=" + stateName);
        this->postEvent<EvPauseRequested>();
      }
      return;
    }

    if (stateName.find("StPause") != std::string::npos)
    {
      if (normalizedKey == 'r')
      {
        this->postEvent<EvKeyResume>();
      }
      else if (normalizedKey == 'b')
      {
        this->postEvent<EvKeyBack>();
      }
      else if (normalizedKey == 'f')
      {
        this->getStateMachine()->setGlobalSMData(
          std::string(sm_data::kPauseReason),
          std::string("[") + log_utils::bjtNowString() + "] 故障转BACK: key='f', state=" + stateName);
        this->postEvent<EvFaultToBack>();
      }
    }
  }

private:
  template <typename TOrthogonal>
  void stopMoveGroupForOrthogonalIfAvailable()
  {
    auto * stateMachine = this->getStateMachine();
    if (stateMachine == nullptr)
    {
      return;
    }

    auto * orthogonal = stateMachine->template getOrthogonal<TOrthogonal>();
    if (orthogonal == nullptr)
    {
      return;
    }

    cl_moveit2z::ClMoveit2z * moveitClient = nullptr;
    if (!orthogonal->requiresClient(moveitClient) || moveitClient == nullptr ||
      moveitClient->moveGroupClientInterface == nullptr)
    {
      return;
    }

    try
    {
      moveitClient->moveGroupClientInterface->stop();
    }
    catch (const std::exception & e)
    {
      RCLCPP_WARN(
        getLogger(),
        "Failed to stop MoveIt client for pause request: %s",
        e.what());
    }
  }

  void cancelTrajectoryExecutorIfAvailable()
  {
    // Pause must stop whichever MoveIt client is active; this component lives on the
    // keyboard orthogonal, so local requiresComponent() cannot see arm executors.
    stopMoveGroupForOrthogonalIfAvailable<OrBothArms>();
    stopMoveGroupForOrthogonalIfAvailable<OrLeftArm>();
    stopMoveGroupForOrthogonalIfAvailable<OrRightArm>();
  }

  bool isPauseDebounced()
  {
    const auto now = std::chrono::steady_clock::now();
    if (lastPauseRequestTime_.time_since_epoch().count() != 0)
    {
      const auto elapsedMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - lastPauseRequestTime_).count();
      if (elapsedMs < 500)
      {
        return true;
      }
    }

    lastPauseRequestTime_ = now;
    return false;
  }

  void request_inspection(
    const std::string & waitStateName,
    const rclcpp::Client<common::srv::InspectCapture>::SharedPtr & client,
    const std::string & serviceName,
    const char * inspectPhase)
  {
    armedInspectWaitState_ = waitStateName;

    if (inspectRequestTimer_)
    {
      inspectRequestTimer_->cancel();
      inspectRequestTimer_.reset();
    }

    inspectRequestTimer_ = this->getNode()->create_wall_timer(
      std::chrono::milliseconds(10),
      [this, waitStateName, client, serviceName, inspectPhase]()
      {
        if (inspectRequestTimer_)
        {
          inspectRequestTimer_->cancel();
          inspectRequestTimer_.reset();
        }

        auto * currentState = this->getStateMachine()->getCurrentState();
        if (currentState == nullptr)
        {
          RCLCPP_WARN(getLogger(), "Cannot request %s inspection: current state is nullptr", inspectPhase);
          return;
        }

        const std::string currentStateName = currentState->getClassName();
        if (currentStateName != waitStateName)
        {
          RCLCPP_WARN(
            log_utils::bizLogger(),
            "[%s] INSPECT %s request skipped: current state=%s no longer matches armed wait state=%s",
            log_utils::bjtNowString().c_str(),
            inspectPhase,
            currentStateName.c_str(),
            waitStateName.c_str());
          return;
        }

        if (serviceName.empty())
        {
          RCLCPP_WARN(
            log_utils::bizLogger(),
            "[%s] INSPECT %s request skipped: service name is empty, waiting for topic/manual ack in state=%s",
            log_utils::bjtNowString().c_str(),
            inspectPhase,
            armedInspectWaitState_.c_str());
          return;
        }

        if (client == nullptr)
        {
          RCLCPP_WARN(
            log_utils::bizLogger(),
            "[%s] INSPECT %s request skipped: client is null for service=%s, waiting for topic/manual ack in state=%s",
            log_utils::bjtNowString().c_str(),
            inspectPhase,
            serviceName.c_str(),
            armedInspectWaitState_.c_str());
          return;
        }

        if (!client->service_is_ready())
        {
          RCLCPP_WARN(
            log_utils::bizLogger(),
            "[%s] INSPECT %s request skipped: service not ready (%s), waiting for topic/manual ack in state=%s",
            log_utils::bjtNowString().c_str(),
            inspectPhase,
            serviceName.c_str(),
            armedInspectWaitState_.c_str());
          return;
        }

        auto request = std::make_shared<common::srv::InspectCapture::Request>();
        const auto requestedState = armedInspectWaitState_;
        client->async_send_request(
          request,
          [this, inspectPhase, serviceName, requestedState](rclcpp::Client<common::srv::InspectCapture>::SharedFuture future)
          {
            try
            {
              const auto response = future.get();
              if (response != nullptr)
              {
                RCLCPP_INFO(
                  log_utils::bizLogger(),
                  "[%s] INSPECT %s response from service=%s for state=%s capture_success=%d quality_good=%d",
                  log_utils::bjtNowString().c_str(),
                  inspectPhase,
                  serviceName.c_str(),
                  requestedState.c_str(),
                  response->capture_success,
                  response->quality_good);

                if (response->capture_success)
                {
                  // quality_good is advisory for now; continue once capture itself succeeded.
                  if (!response->quality_good)
                  {
                    RCLCPP_WARN(
                      log_utils::bizLogger(),
                      "[%s] INSPECT %s capture succeeded but quality_good=false for state=%s; continuing",
                      log_utils::bjtNowString().c_str(),
                      inspectPhase,
                      requestedState.c_str());
                  }
                  handleInspectDoneSignal("service");
                }
                else
                {
                  RCLCPP_WARN(
                    log_utils::bizLogger(),
                    "[%s] INSPECT %s capture failed for state=%s; waiting for retry/topic/manual ack",
                    log_utils::bjtNowString().c_str(),
                    inspectPhase,
                    requestedState.c_str());
                }
              }
              else
              {
                RCLCPP_WARN(
                  log_utils::bizLogger(),
                  "[%s] INSPECT %s request returned null response from service=%s for state=%s; waiting for topic/manual ack",
                  log_utils::bjtNowString().c_str(),
                  inspectPhase,
                  serviceName.c_str(),
                  requestedState.c_str());
              }
            }
            catch (const std::exception & e)
            {
              RCLCPP_WARN(
                log_utils::bizLogger(),
                "[%s] INSPECT %s request failed for service=%s state=%s: %s",
                log_utils::bjtNowString().c_str(),
                inspectPhase,
                serviceName.c_str(),
                requestedState.c_str(),
                e.what());
            }
          });

        RCLCPP_INFO(
          log_utils::bizLogger(),
          "[%s] INSPECT %s request sent to service=%s while waiting in state=%s",
          log_utils::bjtNowString().c_str(),
          inspectPhase,
          serviceName.c_str(),
          requestedState.c_str());
      });
  }

  void handleInspectDoneSignal(const char * source)
  {
    auto * currentState = this->getStateMachine()->getCurrentState();
    if (currentState == nullptr)
    {
      return;
    }

    const std::string stateName = currentState->getClassName();
    if (!armedInspectWaitState_.empty() && stateName == armedInspectWaitState_)
    {
      armedInspectWaitState_.clear();
      if (inspectRequestTimer_)
      {
        inspectRequestTimer_->cancel();
        inspectRequestTimer_.reset();
      }
      RCLCPP_INFO(
        log_utils::bizLogger(),
        "[%s] INSPECT completion acknowledged by %s in state=%s -> post EvInspectStepAck",
        log_utils::bjtNowString().c_str(),
        source,
        stateName.c_str());
      this->postEvent<EvInspectStepAck>();
    }
  }

  void publishPcbDetection(bool present)
  {
    common::msg::PcbDetection detectionMsg;
    detectionMsg.present = present;
    detectionMsg.pose.header.stamp = this->getNode()->now();
    detectionMsg.pose.header.frame_id = "base_link";
    detectionMsg.pose.pose.orientation.w = 1.0;

    if (present)
    {
      // Load pick coordinates from YAML as default
      try
      {
        const std::string packageShareDir =
          ament_index_cpp::get_package_share_directory("je_arm_pcb_inspection_sm");
        const std::string yamlPath =
          packageShareDir + "/config/move_group_client/cartesian_states/pick.yaml";

        YAML::Node root = YAML::LoadFile(yamlPath);

        detectionMsg.pose.header.frame_id =
          root["frame_id"] ? root["frame_id"].as<std::string>() : std::string("base_link");
        detectionMsg.pose.pose.position.x = root["x"].as<double>();
        detectionMsg.pose.pose.position.y = root["y"].as<double>();
        detectionMsg.pose.pose.position.z = root["z"].as<double>();
        detectionMsg.pose.pose.orientation.x = root["qx"].as<double>();
        detectionMsg.pose.pose.orientation.y = root["qy"].as<double>();
        detectionMsg.pose.pose.orientation.z = root["qz"].as<double>();
        detectionMsg.pose.pose.orientation.w = root["qw"].as<double>();
      }
      catch (const std::exception & e)
      {
        RCLCPP_WARN(
          getLogger(),
          "Failed to load pick.yaml: %s, using default values",
          e.what());
        detectionMsg.pose.pose.position.x = -0.273;
        detectionMsg.pose.pose.position.y = 0.0;
        detectionMsg.pose.pose.position.z = 0.412;
        detectionMsg.pose.pose.orientation.x = 0.0;
        detectionMsg.pose.pose.orientation.y = -0.764357;
        detectionMsg.pose.pose.orientation.z = 0.0;
        detectionMsg.pose.pose.orientation.w = 0.644794;
      }
    }

    if (pcbDetectionPub_ != nullptr)
    {
      pcbDetectionPub_->publish(detectionMsg);
    }

    if (present)
    {
      RCLCPP_INFO(
        log_utils::bizLogger(),
        "[%s] KEY 'w': published pcb_detection present=true pose=(%.4f, %.4f, %.4f)",
        log_utils::bjtNowString().c_str(),
        detectionMsg.pose.pose.position.x,
        detectionMsg.pose.pose.position.y,
        detectionMsg.pose.pose.position.z);
    }
    else
    {
      publishPlaceSlot(false);
      RCLCPP_INFO(
        log_utils::bizLogger(),
        "[%s] KEY 'u': published pcb_detection present=false and place_slot free=false",
        log_utils::bjtNowString().c_str());
    }
  }

  void publishInspectDone()
  {
    std_msgs::msg::Bool msg;
    msg.data = true;

    if (inspectDonePub_ != nullptr)
    {
      inspectDonePub_->publish(msg);
    }

    RCLCPP_INFO(
      log_utils::bizLogger(),
      "[%s] KEY 'n': published inspect_done=true",
      log_utils::bjtNowString().c_str());
  }

  void publishFreePlaceSlotFromYaml()
  {
    try
    {
      const std::string packageShareDir =
        ament_index_cpp::get_package_share_directory("je_arm_pcb_inspection_sm");
      const std::string yamlPath =
        packageShareDir + "/config/move_group_client/cartesian_states/place.yaml";

      YAML::Node root = YAML::LoadFile(yamlPath);

      common::msg::PlaceSlot slotMsg;
      slotMsg.free = true;
      slotMsg.pose.header.stamp = this->getNode()->now();
      slotMsg.pose.header.frame_id =
        root["frame_id"] ? root["frame_id"].as<std::string>() : std::string("base_link");
      slotMsg.pose.pose.position.x = root["x"].as<double>();
      slotMsg.pose.pose.position.y = root["y"].as<double>();
      slotMsg.pose.pose.position.z = root["z"].as<double>();
      slotMsg.pose.pose.orientation.x = root["qx"].as<double>();
      slotMsg.pose.pose.orientation.y = root["qy"].as<double>();
      slotMsg.pose.pose.orientation.z = root["qz"].as<double>();
      slotMsg.pose.pose.orientation.w = root["qw"].as<double>();

      if (placeSlotPub_ != nullptr)
      {
        placeSlotPub_->publish(slotMsg);
      }

      RCLCPP_INFO(
        log_utils::bizLogger(),
        "[%s] KEY 'h': published place_slot free=true pose=(%.4f, %.4f, %.4f)",
        log_utils::bjtNowString().c_str(),
        slotMsg.pose.pose.position.x,
        slotMsg.pose.pose.position.y,
        slotMsg.pose.pose.position.z);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        log_utils::bizLogger(),
        "[%s] KEY 'h': failed to load/publish place slot pose: %s",
        log_utils::bjtNowString().c_str(),
        e.what());
    }
  }

  void publishPlaceSlot(bool free)
  {
    common::msg::PlaceSlot slotMsg;
    slotMsg.free = free;
    slotMsg.pose.header.stamp = this->getNode()->now();
    slotMsg.pose.header.frame_id = "base_link";
    slotMsg.pose.pose.orientation.w = 1.0;

    if (placeSlotPub_ != nullptr)
    {
      placeSlotPub_->publish(slotMsg);
    }
  }

  std::string pcbDetectionTopic_{"/pcb_detection"};
  std::string placeSlotTopic_{"/vision/place_slot_detection"};
  std::string inspectDoneTopic_{"/vision/inspect_done"};
  std::string inspectFrontTriggerService_{"/vision/inspect_front"};
  std::string inspectBackTriggerService_{"/vision/inspect_back"};
  rclcpp::Publisher<common::msg::PcbDetection>::SharedPtr pcbDetectionPub_;
  rclcpp::Publisher<common::msg::PlaceSlot>::SharedPtr placeSlotPub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr inspectDonePub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr inspectDoneSub_;
  rclcpp::Client<common::srv::InspectCapture>::SharedPtr inspectFrontTriggerClient_;
  rclcpp::Client<common::srv::InspectCapture>::SharedPtr inspectBackTriggerClient_;
  rclcpp::TimerBase::SharedPtr inspectRequestTimer_;
  std::string armedInspectWaitState_;
  std::chrono::steady_clock::time_point lastPauseRequestTime_{};
};

}  // namespace je_arm_pcb_inspection_sm