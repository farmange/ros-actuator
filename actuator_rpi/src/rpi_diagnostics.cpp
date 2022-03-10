//============================================================================
// Name        : armms_diagnostics.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================
#include <fstream>

#include "actuator_rpi/rpi_diagnostics.h"
#include "rclcpp/duration.hpp"
#include <wiringPi.h>

RpiDiagnostics::RpiDiagnostics(rclcpp_lifecycle::LifecycleNode* node, rclcpp::CallbackGroup::SharedPtr group)
  : node_(node)
{
  cpu_temperature_ = 0;
  loop_rate_ = 0;

  init_parameters_();

  pinMode(probe_temp_pin_, INPUT);
  pullUpDnControl(probe_temp_pin_, PUD_UP);

  if (loop_rate_ <= 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "Bad sampling frequency value (%d)", loop_rate_);
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Initialize timer for rpi diag");
  std::chrono::duration<double> period(1.0 / loop_rate_);
  diag_non_rt_loop_ = node_->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                               std::bind(&RpiDiagnostics::update_, this), group);

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
  const std::lock_guard<std::mutex> lock(mutex_);
  return cpu_temperature_;
}

std::vector<float> RpiDiagnostics::getProbeTemperature()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return probe_temperature_vect_;
}

void RpiDiagnostics::init_parameters_()
{
  node_->declare_parameter<int>("rpi_diag_loop_rate", 1);
  node_->declare_parameter<int>("probe_temp_pin", 4);

  node_->get_parameter("rpi_diag_loop_rate", loop_rate_);
  node_->get_parameter("probe_temp_pin", probe_temp_pin_);
}

void RpiDiagnostics::read_cpu_temperature_()
{
  float cpu_temp, probe_temp;
  std::vector<float> probe_temp_vect;
  std::fstream temp_fstream;
  temp_fstream.open("/sys/class/thermal/thermal_zone0/temp", std::ios_base::in);
  temp_fstream >> cpu_temp;
  cpu_temp = cpu_temp / 1000.0;
  temp_fstream.close();

  std::ifstream infile1wire;
  std::string wire1_slaves_file = "/sys/bus/w1/devices/w1_bus_master1/w1_master_slaves";
  infile1wire.open(wire1_slaves_file.c_str());
  if (infile1wire.is_open())
  {
    std::string device_id;
    while (getline(infile1wire, device_id))
    {
      temp_fstream.open("/sys/bus/w1/devices/" + device_id + "/temperature", std::ios_base::in);
      temp_fstream >> probe_temp;
      probe_temp = probe_temp / 1000.0;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "Read probe (id: " << device_id << ") : " << probe_temp);
      probe_temp_vect.push_back(probe_temp);
      temp_fstream.close();
    }
    infile1wire.close();
  }

  const std::lock_guard<std::mutex> lock(mutex_);
  if (cpu_temp > 0)
  {
    cpu_temperature_ = cpu_temp;
  }
  if (probe_temp > 0)
  {
    probe_temperature_vect_ = probe_temp_vect;
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
