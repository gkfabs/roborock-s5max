#ifndef ROBOROCK_S5MAX__S5MAX_CHARGING_DOCK_HPP_
#define ROBOROCK_S5MAX__S5MAX_CHARGING_DOCK_HPP_

#include <memory>
#include <string>

#include "opennav_docking/simple_charging_dock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "vacuum_ros2_bridge/msg/vacuum_status.hpp"
#include "vacuum_ros2_bridge/srv/set_actuator.hpp"

namespace roborock_s5max
{

class S5MaxChargingDock : public opennav_docking::SimpleChargingDock
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    std::shared_ptr<tf2_ros::Buffer> tf) override;

  bool isDocked() override;
  bool isCharging() override;
  bool disableCharging() override;
  bool hasStoppedCharging() override;

private:
  using SetActuator = vacuum_ros2_bridge::srv::SetActuator;
  using VacuumStatus = vacuum_ros2_bridge::msg::VacuumStatus;

  bool statusIsFresh() const;
  void requestCharger(bool enabled);
  void statusCallback(const VacuumStatus::SharedPtr message);
  void leftBumperCallback(const std_msgs::msg::Bool::SharedPtr message);
  void rightBumperCallback(const std_msgs::msg::Bool::SharedPtr message);

  rclcpp::Subscription<VacuumStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_bumper_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_bumper_subscription_;
  rclcpp::Client<SetActuator>::SharedPtr charger_client_;
  rclcpp::Client<SetActuator>::SharedFuture charger_future_;
  rclcpp::Time status_received_at_;
  bool have_status_{false};
  bool is_docked_{false};
  bool is_charging_{false};
  bool charger_target_{false};
  bool charger_request_target_{false};
  bool charger_request_active_{false};
  bool left_bumper_{false};
  bool right_bumper_{false};
  bool bumper_contact_available_{false};
  bool dock_contact_candidate_{false};
  bool contact_reported_{false};
  double status_timeout_sec_{0.5};
  int charger_speed_{1};
};

}  // namespace roborock_s5max

#endif  // ROBOROCK_S5MAX__S5MAX_CHARGING_DOCK_HPP_
