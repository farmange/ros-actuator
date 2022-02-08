//============================================================================
// Name        : rpi_switch_limit.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "actuator_rpi/rpi_switch_limit.h"

RpiSwitchLimit::RpiSwitchLimit(rclcpp_lifecycle::LifecycleNode* node) : node_(node)
{
  init_parameters_();

  pinMode(switch_limit_pin_, INPUT);
  pullUpDnControl(switch_limit_pin_, PUD_OFF);
}

void RpiSwitchLimit::update(bool& switch_limit)
{
  /* Low side commutation and normally closed contact*/
  switch_limit = digitalRead(switch_limit_pin_);
}

void RpiSwitchLimit::init_parameters_()
{
  node_->declare_parameter<int>("rpi_switch_limit_pin", 0);
  node_->get_parameter("rpi_switch_limit_pin", switch_limit_pin_);
}
