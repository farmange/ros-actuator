//============================================================================
// Name        : rpi_switch_limit.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : Class definition that bind all low level function
//               of the libkinovadrv
//============================================================================

#ifndef ACTUATOR_RPI_RPI_SWITCH_LIMIT_H
#define ACTUATOR_RPI_RPI_SWITCH_LIMIT_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <wiringPi.h>

class RpiSwitchLimit
{
public:
  RpiSwitchLimit(rclcpp_lifecycle::LifecycleNode* node);
  void update(bool& switch_limit);

private:
  rclcpp_lifecycle::LifecycleNode* node_;

  int switch_limit_pin_;

  void init_parameters_();
};

#endif
