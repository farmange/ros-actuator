//============================================================================
// Name        : rpi_user_button.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : Class definition that bind all low level function
//               of the libkinovadrv
//============================================================================

#ifndef ACTUATOR_RPI_RPI_USER_BUTTON_H
#define ACTUATOR_RPI_RPI_USER_BUTTON_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "actuator_msgs/srv/button_mode_event.hpp"
#include <wiringPi.h>

class RpiUserButton
{
  using ButtonModeEventSrv = actuator_msgs::srv::ButtonModeEvent;

public:
  RpiUserButton(rclcpp_lifecycle::LifecycleNode* node);
  void update(bool& button_up, bool& button_down, bool& button_mode);

private:
  rclcpp_lifecycle::LifecycleNode* node_;

  int btn_up_pin_;
  int btn_down_pin_;
  int btn_mode_pin_;

  void init_parameters_();
  void init_services_();
  
  void process_button_mode_state_(const bool& button_mode);

  ///////////////////////////////////
  rclcpp::Client<ButtonModeEventSrv>::SharedPtr button_mode_srv_client_;

  bool button_mode_prev_;
  bool long_press_detected_;

  double long_press_duration_;
  double inactivity_duration_;
  rclcpp::Time press_time_;
  rclcpp::Time release_time_;
  int press_counter_;
  ///////////////////
};

#endif
