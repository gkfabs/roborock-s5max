#include "roborock_s5max/s5max_charging_dock.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace roborock_s5max
{

void S5MaxChargingDock::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name,
  std::shared_ptr<tf2_ros::Buffer> tf)
{
  opennav_docking::SimpleChargingDock::configure(parent, name, tf);
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("The docking server node is unavailable");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".status_timeout_sec", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".charger_speed", rclcpp::ParameterValue(1));
  node->get_parameter(name + ".status_timeout_sec", status_timeout_sec_);
  node->get_parameter(name + ".charger_speed", charger_speed_);
  charger_speed_ = std::clamp(charger_speed_, 1, 100);

  status_received_at_ = node->now();
  status_subscription_ = node->create_subscription<VacuumStatus>(
    "vacuum_status", rclcpp::SensorDataQoS(),
    std::bind(&S5MaxChargingDock::statusCallback, this, std::placeholders::_1));
  auto bumper_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  left_bumper_subscription_ = node->create_subscription<std_msgs::msg::Bool>(
    "/bumper/left", bumper_qos,
    std::bind(&S5MaxChargingDock::leftBumperCallback, this, std::placeholders::_1));
  right_bumper_subscription_ = node->create_subscription<std_msgs::msg::Bool>(
    "/bumper/right", bumper_qos,
    std::bind(&S5MaxChargingDock::rightBumperCallback, this, std::placeholders::_1));
  charger_client_ = node->create_client<SetActuator>("set_actuator");
}

void S5MaxChargingDock::statusCallback(const VacuumStatus::SharedPtr message)
{
  status_received_at_ = node_->now();
  have_status_ = true;
  is_docked_ = message->is_docked;
  is_charging_ = message->is_charging;
}

void S5MaxChargingDock::leftBumperCallback(const std_msgs::msg::Bool::SharedPtr message)
{
  left_bumper_ = message->data;
  if (!left_bumper_ && !right_bumper_) {
    contact_reported_ = false;
  }
}

void S5MaxChargingDock::rightBumperCallback(const std_msgs::msg::Bool::SharedPtr message)
{
  right_bumper_ = message->data;
  if (!left_bumper_ && !right_bumper_) {
    contact_reported_ = false;
  }
}

bool S5MaxChargingDock::statusIsFresh() const
{
  if (!have_status_) {
    return false;
  }
  return (node_->now() - status_received_at_).seconds() <= status_timeout_sec_;
}

void S5MaxChargingDock::requestCharger(bool enabled)
{
  if (charger_request_active_) {
    if (charger_future_.valid() &&
      charger_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      const auto response = charger_future_.get();
      charger_request_active_ = false;
      if (!response->success) {
        RCLCPP_WARN(node_->get_logger(), "The charger request failed: %s", response->message.c_str());
      }
    }
    return;
  }
  if (!charger_client_->service_is_ready()) {
    return;
  }
  auto request = std::make_shared<SetActuator::Request>();
  request->component = "charger";
  request->speed = enabled ? static_cast<uint8_t>(charger_speed_) : 0;
  charger_target_ = enabled;
  auto future_and_id = charger_client_->async_send_request(request);
  charger_future_ = future_and_id.future.share();
  charger_request_active_ = true;
}

bool S5MaxChargingDock::isDocked()
{
  if (left_bumper_ || right_bumper_) {
    if (!contact_reported_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Bumper contact stopped the dock approach. Waiting for charging confirmation.");
      contact_reported_ = true;
    }
    return true;
  }
  if (statusIsFresh() && is_docked_) {
    return true;
  }
  return opennav_docking::SimpleChargingDock::isDocked();
}

bool S5MaxChargingDock::isCharging()
{
  if (statusIsFresh() && is_charging_) {
    return true;
  }
  if (statusIsFresh() && is_docked_) {
    requestCharger(true);
  }
  return false;
}

bool S5MaxChargingDock::disableCharging()
{
  requestCharger(false);
  if (charger_request_active_) {
    return false;
  }
  return !charger_target_;
}

bool S5MaxChargingDock::hasStoppedCharging()
{
  return statusIsFresh() && !is_charging_;
}

}  // namespace roborock_s5max

PLUGINLIB_EXPORT_CLASS(
  roborock_s5max::S5MaxChargingDock,
  opennav_docking_core::ChargingDock)
