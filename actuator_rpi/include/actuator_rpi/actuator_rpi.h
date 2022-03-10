//============================================================================
// Name        : actuator_rpi.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : Class definition that bind all low level function
//               of the libkinovadrv
//============================================================================

#ifndef ACTUATOR_RPI_ACTUATOR_RPI_H
#define ACTUATOR_RPI_ACTUATOR_RPI_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <wiringPi.h>
// #include <softPwm.h>

#include "actuator_msgs/msg/rpi_interface.hpp"
#include "actuator_rpi/rpi_switch_limit.h"
// #include "armms_rpi/armms_power_button_led.h"
#include "actuator_rpi/rpi_diagnostics.h"
#include "actuator_rpi/rpi_user_button.h"
// #include "armms_rpi/armms_motor_power.h"
// #include "armms_rpi/armms_shutdown_manager.h"

class ActuatorRpi : public rclcpp_lifecycle::LifecycleNode
{
public:
  ActuatorRpi(const std::string& node_name, bool intra_process_comms = false);

private:
  rclcpp_lifecycle::LifecyclePublisher<actuator_msgs::msg::RpiInterface>::SharedPtr rpi_interface_pub_;
  std::shared_ptr<RpiUserButton> user_button_;
  std::shared_ptr<RpiSwitchLimit> switch_limit_;
  std::shared_ptr<RpiDiagnostics> rpi_diagnostics_;

  rclcpp::TimerBase::SharedPtr timer_;
  int rpi_loop_rate_;
  rclcpp::CallbackGroup::SharedPtr callback_group_thread1_;
  rclcpp::CallbackGroup::SharedPtr callback_group_thread2_;
  void init_parameters_();
  void init_publishers_();
  void clear_publishers_();

  void update_();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State&);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State&);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State&);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State&);
};

#endif
