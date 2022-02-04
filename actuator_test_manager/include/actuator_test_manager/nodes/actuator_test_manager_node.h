//============================================================================
// Name        : actuator_test_manager.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================
#ifndef ACTUATOR_TEST_MANAGER_H
#define ACTUATOR_TEST_MANAGER_H

// #include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"
#include "actuator_msgs/msg/actuator_state.hpp"
#include "actuator_msgs/msg/control_mode.hpp"
#include "actuator_msgs/srv/set_control_mode.hpp"

// #include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
// #include "rclcpp_lifecycle/lifecycle_publisher.hpp"

// using rclcpp_lifecycle::node_interfaces;

class ActuatorTestManager : public rclcpp_lifecycle::LifecycleNode
{
  using ControlModeMsg = actuator_msgs::msg::ControlMode;
  using SetControlModeSrv = actuator_msgs::srv::SetControlMode;

public:
  ActuatorTestManager(const std::string& node_name, bool intra_process_comms = false);

  ~ActuatorTestManager();

  CallbackReturn on_configure(const rclcpp_lifecycle::State&);

  CallbackReturn on_activate(const rclcpp_lifecycle::State&);

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&);

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&);

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&);

private:
  rclcpp::Subscription<actuator_msgs::msg::ActuatorState>::SharedPtr actuator_state_sub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr position_command_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr torque_command_pub_;
  rclcpp::Client<actuator_msgs::srv::SetControlMode>::SharedPtr set_control_mode_srvclient_;

  actuator_msgs::msg::ActuatorState actuator_state_;
  std_msgs::msg::Float32 position_command_;
  std_msgs::msg::Float32 torque_command_;

private:
  void init_publishers_();
  void init_subscribers_();
  void init_services_();
  void actuator_state_cb_(const actuator_msgs::msg::ActuatorState::SharedPtr msg);
  void control_loop_cb_();

private:
  rclcpp::TimerBase::SharedPtr timer_;
};
#endif