//============================================================================
// Name        : armms_diagnostics.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : Class definition that bind all low level function
//               of the libkinovadrv
//============================================================================

#ifndef ACTUATOR_RPI_RPI_DIAGNOSTICS_H
#define ACTUATOR_RPI_RPI_DIAGNOSTICS_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class RpiDiagnostics
{
public:
  RpiDiagnostics(rclcpp_lifecycle::LifecycleNode* node);
  void start();
  void stop();
  float getCpuTemperature();

private:
  rclcpp_lifecycle::LifecycleNode* node_;
  rclcpp::TimerBase::SharedPtr diag_non_rt_loop_;

  float cpu_temperature_;
  int loop_rate_;
  void init_parameters_();
  void read_cpu_temperature_();
  void update_();
};

#endif
