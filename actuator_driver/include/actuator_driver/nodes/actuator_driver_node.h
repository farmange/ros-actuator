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
#include "std_srvs/srv/trigger.hpp"

#include "actuator_driver/base_comm.h"
#include "actuator_driver/actuator_hardware_interface.h"

class ActuatorDriver : public rclcpp::Node
{
public:
  ActuatorDriver();
  void rosControlLoop();

private:
  void positionCommandCallback_(const std_msgs::msg::Float32::SharedPtr msg);
  void stopSrvCallback_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void startSrvCallback_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void retrieveParameters_();
  void initializeServices_();
  void initializePublisher_();
  void initializeSubscriber_();

private:
  std::shared_ptr<BaseComm> comm_;
  std::shared_ptr<ActuatorHardwareInterface> actuator_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr position_command_sub_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorState>::SharedPtr actuator_state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  actuator_msgs::msg::ActuatorState actuator_state_;

  float position_command_;
  bool motor_on_;
};

#endif