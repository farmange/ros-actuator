//============================================================================
// Name        : actuator_driver.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "actuator_driver/nodes/actuator_driver_node.h"
#include "actuator_driver/rmd_comm.h"
#include <chrono>
#include <unistd.h>
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

ActuatorDriver::ActuatorDriver(const std::string& node_name, bool intra_process_comms)
  : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
  this->declare_parameter<std::string>("can_device", "can0");
  this->declare_parameter<int>("actuator_reduction_ratio", 36);
  this->declare_parameter<int>("actuator_max_speed", 1080);
  this->declare_parameter<int>("actuator_max_accel", 700);
  this->declare_parameter<double>("actuator_current_limit", 0.0);

  control_mode_.id = ControlModeMsg::ID_MODE_OFF;
}

ActuatorDriver::~ActuatorDriver()
{
  RCLCPP_DEBUG(this->get_logger(), "Destructor of ActuatorDriver");
  if (actuator_ != nullptr)
  {
    timer_->cancel();
    actuator_stop_();
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ActuatorDriver::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "ActuatorDriver::on_configure");

  init_parameters_();
  init_services_();
  init_publisher_();
  init_subscriber_();

  comm_ = std::make_shared<RMDComm>(this);
  BaseComm::CommStatus_t init_result = comm_->init(can_device_param_, static_cast<uint8_t>(reduction_ratio_param_),
                                                   static_cast<uint16_t>(max_speed_param_),
                                                   static_cast<uint32_t>(max_accel_param_), current_limit_param_);

  if (init_result != BaseComm::COMM_STATUS_OK)
  {
    RCLCPP_ERROR(get_logger(), "Cannot initialize communication instance...");
    // armms_msgs::SetInt msgShutdown;
    // msgShutdown.request.value = 1;  // shutdown
    // ros::shutdown();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(get_logger(), "ARMMS communication has been successfully started");

  RCLCPP_INFO(get_logger(), "Initialize hardware interface");
  actuator_ = std::make_shared<ActuatorHardwareInterface>(this, comm_);
  actuator_stop_();

  RCLCPP_INFO(get_logger(), "Initialize timer for control loop");
  timer_ = this->create_wall_timer(10ms, std::bind(&ActuatorDriver::control_loop_cb_, this));
  timer_->cancel();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ActuatorDriver::on_activate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "ActuatorDriver::on_activate");
  /*TODO activate publisher, sub, timer */
  RCLCPP_INFO(get_logger(), "Starting ros control thread...");
  actuator_state_pub_->on_activate();
  timer_->reset();
  RCLCPP_INFO(get_logger(), "Timer is now reseted");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ActuatorDriver::on_deactivate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "ActuatorDriver::on_deactivate");
  timer_->cancel();
  actuator_stop_();
  actuator_state_pub_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ActuatorDriver::on_cleanup(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "ActuatorDriver::on_cleanup");

  if (actuator_ != nullptr)
  {
    timer_->cancel();
    actuator_stop_();
  }

  actuator_.reset();
  comm_.reset();
  timer_.reset();

  clear_services_();
  clear_publisher_();
  clear_subscriber_();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ActuatorDriver::on_shutdown(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "ActuatorDriver::on_shutdown");
  actuator_stop_();
  actuator_state_pub_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void ActuatorDriver::init_parameters_()
{
  this->get_parameter("can_device", can_device_param_);
  this->get_parameter("actuator_reduction_ratio", reduction_ratio_param_);
  this->get_parameter("actuator_max_speed", max_speed_param_);
  this->get_parameter("actuator_max_accel", max_accel_param_);
  this->get_parameter("actuator_current_limit", current_limit_param_);

  RCLCPP_DEBUG(get_logger(), "can_device : %s", can_device_param_.c_str());
  RCLCPP_DEBUG(get_logger(), "actuator_reduction_ratio : %d", reduction_ratio_param_);
  RCLCPP_DEBUG(get_logger(), "actuator_max_speed : %d", max_speed_param_);
  RCLCPP_DEBUG(get_logger(), "actuator_max_accel : %d", max_accel_param_);
  RCLCPP_DEBUG(get_logger(), "actuator_current_limit : %f", current_limit_param_);
}

void ActuatorDriver::init_services_()
{
  set_control_mode_srv_ = this->create_service<actuator_msgs::srv::SetControlMode>(
      "set_control_mode", std::bind(&ActuatorDriver::set_control_mode_srv_cb_, this, _1, _2));
}

void ActuatorDriver::init_publisher_()
{
  actuator_state_pub_ = this->create_publisher<actuator_msgs::msg::ActuatorState>("actuator_state", 1);
}

void ActuatorDriver::init_subscriber_()
{
  position_command_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "position_command", 1, std::bind(&ActuatorDriver::position_command_cb_, this, _1));
  torque_command_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "torque_command", 1, std::bind(&ActuatorDriver::torque_command_cb_, this, _1));
}

void ActuatorDriver::clear_services_()
{
  set_control_mode_srv_.reset();
}

void ActuatorDriver::clear_publisher_()
{
  actuator_state_pub_.reset();
}

void ActuatorDriver::clear_subscriber_()
{
  position_command_sub_.reset();
  torque_command_sub_.reset();
}

void ActuatorDriver::control_loop_cb_()
{
  // RCLCPP_INFO(get_logger(), "ActuatorDriver::control_loop_cb_");
  actuator_->read();

  if (control_mode_.id == ControlModeMsg::ID_MODE_OFF)
  {
    actuator_->stop();
  }
  else if (control_mode_.id == ControlModeMsg::ID_MODE_TORQUE)
  {
    actuator_->setTorqueCommand(torque_command_);
    actuator_->send_torque_cmd();
  }
  else if (control_mode_.id == ControlModeMsg::ID_MODE_POSITION)
  {
    actuator_->setPositionCommand(position_command_);
    actuator_->send_position_cmd();
  }

  actuator_->getState(actuator_state_);
  actuator_state_pub_->publish(actuator_state_);
  // RCLCPP_INFO(get_logger(), "ActuatorDriver::control_loop_cb_ end...");
}

void ActuatorDriver::position_command_cb_(const std_msgs::msg::Float32::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Position command set to: '%f'", msg->data);
  position_command_ = msg->data;
}

void ActuatorDriver::torque_command_cb_(const std_msgs::msg::Float32::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Torque command set to: '%f'", msg->data);
  torque_command_ = msg->data;
}

void ActuatorDriver::set_control_mode_srv_cb_(
    const std::shared_ptr<actuator_msgs::srv::SetControlMode::Request> request,
    std::shared_ptr<actuator_msgs::srv::SetControlMode::Response> response)
{
  RCLCPP_DEBUG(this->get_logger(), "set_control_mode_srv_cb_");
  if ((request->mode.id == ControlModeMsg::ID_MODE_OFF) || (request->mode.id == ControlModeMsg::ID_MODE_TORQUE) ||
      (request->mode.id == ControlModeMsg::ID_MODE_POSITION))
  {
    control_mode_.id = request->mode.id;
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void ActuatorDriver::actuator_stop_()
{
  control_mode_.id = ControlModeMsg::ID_MODE_OFF;
  actuator_->stop();
}
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  std::shared_ptr<ActuatorDriver> actuator_driver_node = std::make_shared<ActuatorDriver>("actuator_driver_node");
  exe.add_node(actuator_driver_node->get_node_base_interface());
  RCLCPP_INFO(actuator_driver_node->get_logger(), "Start spinning \'%s\'...", actuator_driver_node->get_name());
  exe.spin();
  RCLCPP_INFO(actuator_driver_node->get_logger(), "Shutting down \'%s\'...", actuator_driver_node->get_name());
  rclcpp::shutdown();

  return 0;
}