//============================================================================
// Name        : supporter_controller.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================
#ifndef SUPPORTER_CONTROLLER_H
#define SUPPORTER_CONTROLLER_H

// #include <memory>

#include "rclcpp/rclcpp.hpp"

#include "actuator_msgs/msg/control_command.hpp"
#include "actuator_msgs/msg/control_mode.hpp"
#include "actuator_msgs/msg/rpi_interface.hpp"
#include "actuator_msgs/srv/button_mode_event.hpp"
#include "actuator_msgs/srv/set_rgb_led.hpp"

// #include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
// #include "rclcpp_lifecycle/lifecycle_publisher.hpp"

// using rclcpp_lifecycle::node_interfaces;

class SupporterController : public rclcpp_lifecycle::LifecycleNode
{
  using ControlModeMsg = actuator_msgs::msg::ControlMode;
  using ControlCommandMsg = actuator_msgs::msg::ControlCommand;
  using RpiInterfaceMsg = actuator_msgs::msg::RpiInterface;
  using ButtonModeEventSrv = actuator_msgs::srv::ButtonModeEvent;
  using SetRgbLedSrv = actuator_msgs::srv::SetRgbLed;

public:
  SupporterController(const std::string& node_name, bool intra_process_comms = false);

  ~SupporterController();

  CallbackReturn on_configure(const rclcpp_lifecycle::State&);

  CallbackReturn on_activate(const rclcpp_lifecycle::State&);

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&);

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&);

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&);

private:
  rclcpp::Subscription<RpiInterfaceMsg>::SharedPtr rpi_interface_sub_;
  rclcpp_lifecycle::LifecyclePublisher<ControlCommandMsg>::SharedPtr control_command_pub_;
  rclcpp::Service<ButtonModeEventSrv>::SharedPtr button_mode_event_srv_;
  rclcpp::Client<SetRgbLedSrv>::SharedPtr set_rgb_led_srv_client_;

  rclcpp::TimerBase::SharedPtr timer_;

  RpiInterfaceMsg rpi_interface_;
  ControlCommandMsg control_command_;
  ControlModeMsg control_mode_request_;
  bool enabled_;

  float high_velocity_param_;
  float low_velocity_param_;
  double vel_ramp_duration_param_;
  double low_velocity_duration_param_;
  bool adapt_vel_rising_edge_detected_;
  rclcpp::Time adapt_velocity_time_;

  double low_torque_inc_param_;
  double high_torque_inc_param_;
  double torque_ramp_duration_param_;
  double low_torque_duration_param_;
  bool adapt_torque_rising_edge_detected_;
  rclcpp::Time adapt_torque_time_;
  double torque_increment_;

  int loop_rate_param_;

private:
  void init_parameters_();
  void init_publishers_();
  void init_subscribers_();
  void init_services_();
  void button_mode_event_cb_(const std::shared_ptr<ButtonModeEventSrv::Request> request,
                             std::shared_ptr<ButtonModeEventSrv::Response> response);
  void rpi_interface_cb_(const RpiInterfaceMsg::SharedPtr msg);

  void control_loop_cb_();
  void set_rgb_led_();
  void adapt_velocity_(float& velocity_cmd);
  void adapt_torque_(float& torque_cmd, const double& torque_increment);
};
#endif