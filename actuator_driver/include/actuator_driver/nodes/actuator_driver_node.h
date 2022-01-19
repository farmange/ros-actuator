//============================================================================
// Name        : actuator_driver.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================
#ifndef ACTUATOR_DRIVER_H
#define ACTUATOR_DRIVER_H

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"
#include "actuator_msgs/msg/actuator_state.hpp"
#include "actuator_msgs/msg/control_mode.hpp"
#include "actuator_msgs/srv/set_control_mode.hpp"

#include "actuator_driver/base_comm.h"
#include "actuator_driver/actuator_hardware_interface.h"

// #include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
// #include "rclcpp_lifecycle/lifecycle_publisher.hpp"

// using rclcpp_lifecycle::node_interfaces;

class ActuatorDriver : public rclcpp_lifecycle::LifecycleNode
{
  using ControlModeMsg = actuator_msgs::msg::ControlMode;
  using SetControlModeSrv = actuator_msgs::srv::SetControlMode;

public:
  ActuatorDriver(const std::string& node_name, bool intra_process_comms = false);
  ~ActuatorDriver();

private:
  void control_loop_cb_();

  void position_command_cb_(const std_msgs::msg::Float32::SharedPtr msg);
  void torque_command_cb_(const std_msgs::msg::Float32::SharedPtr msg);
  void set_control_mode_srv_cb_(const std::shared_ptr<SetControlModeSrv::Request> request,
                                std::shared_ptr<SetControlModeSrv::Response> response);
  void init_parameters_();
  void init_services_();
  void init_publisher_();
  void init_subscriber_();

  void clear_services_();
  void clear_publisher_();
  void clear_subscriber_();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State&);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State&);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State&);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State&);

  void actuator_stop_();

private:
  std::shared_ptr<BaseComm> comm_;
  std::shared_ptr<ActuatorHardwareInterface> actuator_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr position_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr torque_command_sub_;

  // rclcpp::Publisher<actuator_msgs::msg::ActuatorState>::SharedPtr actuator_state_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<actuator_msgs::msg::ActuatorState>> actuator_state_pub_;

  rclcpp::Service<SetControlModeSrv>::SharedPtr set_control_mode_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  // std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> lc_pub_;

  actuator_msgs::msg::ActuatorState actuator_state_;

  float position_command_;
  float torque_command_;
  ControlModeMsg control_mode_;

  std::string can_device_param_;
  int reduction_ratio_param_;
  int max_speed_param_;
  int max_accel_param_;
  float current_limit_param_;
};

#endif