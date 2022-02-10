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

#include "actuator_msgs/msg/actuator_state.hpp"
#include "actuator_msgs/msg/control_command.hpp"
#include "actuator_msgs/msg/control_mode.hpp"
#include "actuator_msgs/srv/set_control_mode.hpp"

#include "actuator_driver/base_comm.h"
#include "actuator_driver/actuator_hardware_interface.h"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
// #include "rclcpp_lifecycle/lifecycle_publisher.hpp"

// using rclcpp_lifecycle::node_interfaces;

class ActuatorDriver : public rclcpp_lifecycle::LifecycleNode
{
  using ControlCommandMsg = actuator_msgs::msg::ControlCommand;
  using ControlModeMsg = actuator_msgs::msg::ControlMode;
  using ActuatorStateMsg = actuator_msgs::msg::ActuatorState;
  using SetControlModeSrv = actuator_msgs::srv::SetControlMode;

public:
  ActuatorDriver(const std::string& node_name, bool intra_process_comms = false);
  ~ActuatorDriver();

private:
  void control_loop_cb_();

  void control_command_cb_(const ControlCommandMsg::SharedPtr msg);
  void set_control_mode_srv_cb_(const std::shared_ptr<SetControlModeSrv::Request> request,
                                std::shared_ptr<SetControlModeSrv::Response> response);
  void init_parameters_();
  void init_services_();
  void init_publishers_();
  void init_subscribers_();

  void clear_services_();
  void clear_publishers_();
  void clear_subscribers_();

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

  rclcpp::Subscription<ControlCommandMsg>::SharedPtr control_command_sub_;

  rclcpp_lifecycle::LifecyclePublisher<ActuatorStateMsg>::SharedPtr actuator_state_pub_;

  rclcpp::Service<SetControlModeSrv>::SharedPtr set_control_mode_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  // std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> lc_pub_;

  ActuatorStateMsg actuator_state_;

  ControlCommandMsg control_command_;

  std::string can_device_param_;
  int reduction_ratio_param_;
  int max_speed_param_;
  int max_accel_param_;
  float current_limit_param_;
  int loop_rate_param_;
};

#endif