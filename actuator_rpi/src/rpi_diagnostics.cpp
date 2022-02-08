//============================================================================
// Name        : armms_diagnostics.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================
#include <fstream>

#include "actuator_rpi/rpi_diagnostics.h"
#include "rclcpp/duration.hpp"

RpiDiagnostics::RpiDiagnostics(rclcpp_lifecycle::LifecycleNode* node) : node_(node)
{
  cpu_temperature_ = 0;
  loop_rate_ = 0;

  init_parameters_();

  if (loop_rate_ <= 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "Bad sampling frequency value (%d)", loop_rate_);
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Initialize timer for rpi diag");
  std::chrono::duration<double> period(1.0 / loop_rate_);
  diag_non_rt_loop_ = node_->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                               std::bind(&RpiDiagnostics::update_, this));

  diag_non_rt_loop_->cancel();
}

void RpiDiagnostics::start()
{
  diag_non_rt_loop_->reset();
}

void RpiDiagnostics::stop()
{
  diag_non_rt_loop_->cancel();
}

float RpiDiagnostics::getCpuTemperature()
{
  return cpu_temperature_;
}

void RpiDiagnostics::init_parameters_()
{
  node_->declare_parameter<int>("rpi_diag_loop_rate", 1);
  node_->get_parameter("rpi_diag_loop_rate", loop_rate_);
}

void RpiDiagnostics::read_cpu_temperature_()
{
  std::fstream temp_fstream("/sys/class/thermal/thermal_zone0/temp", std::ios_base::in);

  float cpu_temp;
  temp_fstream >> cpu_temp;
  if (cpu_temp > 0)
  {
    cpu_temperature_ = cpu_temp / 1000.0;
  }
}

void RpiDiagnostics::update_()
{
  read_cpu_temperature_();
  RCLCPP_DEBUG(node_->get_logger(), "Read CPU temperature : %f", cpu_temperature_);

  // check if Rpi is too hot
  if (cpu_temperature_ > 75.0)
  {
    RCLCPP_WARN(node_->get_logger(), "Rpi temperature is really high !");
  }
  if (cpu_temperature_ > 85.0)
  {
    RCLCPP_ERROR(node_->get_logger(), "Rpi is too hot, shutdown to avoid any damage !!!");
  }
}
