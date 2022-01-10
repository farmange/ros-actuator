//============================================================================
// Name        : actuator_driver.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "actuator_driver/nodes/actuator_driver_node.h"
#include "actuator_driver/rmd_comm.h"
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

ActuatorDriver::ActuatorDriver() : Node("ActuatorDriver")
{
  retrieveParameters_();
  initializeServices_();
  initializePublisher_();
  initializeSubscriber_();

  comm_ = std::make_shared<RMDComm>(this);
  // comm_->init(const std::string& can_device, const uint8_t& reduction_ratio, const uint16_t& max_speed, const
  // uint32_t& max_accel);
  std::string can_device = "can0";
  uint8_t reduction_ratio = 36;
  uint16_t max_speed = 1080;
  uint32_t max_accel = 700;
  int init_result = comm_->init(can_device, reduction_ratio, max_speed, max_accel);

  if (init_result != 0)
  {
    RCLCPP_ERROR(get_logger(), "Cannot initialize communication instance...");
    // armms_msgs::SetInt msgShutdown;
    // msgShutdown.request.value = 1;  // shutdown
    // ros::shutdown();
    return;
  }
  RCLCPP_INFO(get_logger(), "ARMMS communication has been successfully started");

  RCLCPP_INFO(get_logger(), "Start hardware interface");
  actuator_ = std::make_shared<ActuatorHardwareInterface>(this, comm_.get());

  RCLCPP_INFO(get_logger(), "Starting ros control thread...");
  timer_ = this->create_wall_timer(10ms, std::bind(&ActuatorDriver::rosControlLoop, this));
}

void ActuatorDriver::retrieveParameters_()
{
}

void ActuatorDriver::initializeServices_()
{
  stop_srv_ =
      this->create_service<std_srvs::srv::Trigger>("/stop", std::bind(&ActuatorDriver::stopSrvCallback_, this, _1, _2));
  start_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/start", std::bind(&ActuatorDriver::startSrvCallback_, this, _1, _2));
}

void ActuatorDriver::initializePublisher_()
{
  actuator_state_pub_ = this->create_publisher<actuator_msgs::msg::ActuatorState>("/actuator_state", 1);
}

void ActuatorDriver::initializeSubscriber_()
{
  position_command_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/position_command", 1, std::bind(&ActuatorDriver::positionCommandCallback_, this, _1));
}

void ActuatorDriver::rosControlLoop()
{
  if (!motor_on_)
  {
    actuator_->stop();
  }
  else
  {
    actuator_->read();
    actuator_->setPositionCommand(position_command_);
    actuator_->write();
  }
  actuator_->getState(actuator_state_);
  actuator_state_pub_->publish(actuator_state_);
}

void ActuatorDriver::positionCommandCallback_(const std_msgs::msg::Float32::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Position command set to: '%f'", msg->data);
  position_command_ = msg->data;
}

void ActuatorDriver::stopSrvCallback_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  motor_on_ = false;
}

void ActuatorDriver::startSrvCallback_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  motor_on_ = true;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActuatorDriver>());
  rclcpp::shutdown();
  return 0;
}