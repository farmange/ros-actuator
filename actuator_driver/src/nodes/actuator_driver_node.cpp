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
  this->declare_parameter<float>("actuator_current_limit", 0.0);

  control_command_.mode.id = ControlModeMsg::ID_MODE_OFF;
  control_command_.position = 0.0;
  control_command_.velocity = 0.0;
  control_command_.torque = 0.0;
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
  init_publishers_();
  init_subscribers_();

  comm_ = std::make_shared<RMDComm>(this);
  BaseComm::CommStatus_t init_result = comm_->init(can_device_param_, static_cast<uint8_t>(reduction_ratio_param_),
                                                   static_cast<uint16_t>(max_speed_param_),
                                                   static_cast<uint32_t>(max_accel_param_), current_limit_param_);

  if (init_result != BaseComm::COMM_STATUS_OK)
  {
    RCLCPP_ERROR(get_logger(), "Cannot initialize communication instance...");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(get_logger(), "ARMMS communication has been successfully started");

  RCLCPP_INFO(get_logger(), "Initialize hardware interface");
  actuator_ = std::make_shared<ActuatorHardwareInterface>(this, comm_, current_limit_param_);
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
  clear_publishers_();
  clear_subscribers_();

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

void ActuatorDriver::init_publishers_()
{
  actuator_state_pub_ = this->create_publisher<ActuatorStateMsg>("actuator_state", 1);
}

void ActuatorDriver::init_subscribers_()
{
  control_command_sub_ = this->create_subscription<ControlCommandMsg>(
      "control_command", 1, std::bind(&ActuatorDriver::control_command_cb_, this, _1));
}

void ActuatorDriver::clear_services_()
{
  set_control_mode_srv_.reset();
}

void ActuatorDriver::clear_publishers_()
{
  actuator_state_pub_.reset();
}

void ActuatorDriver::clear_subscribers_()
{
  control_command_sub_.reset();
}

void ActuatorDriver::control_loop_cb_()
{
  // RCLCPP_INFO(get_logger(), "ActuatorDriver::control_loop_cb_");
  actuator_->read();

  if (control_command_.mode.id == ControlModeMsg::ID_MODE_OFF)
  {
    actuator_->stop();
  }
  else if (control_command_.mode.id == ControlModeMsg::ID_MODE_TORQUE)
  {
    actuator_->set_torque_command(control_command_.torque);
    actuator_->send_torque_cmd();
  }
  else if (control_command_.mode.id == ControlModeMsg::ID_MODE_POSITION)
  {
    actuator_->set_position_command(control_command_.position);
    actuator_->send_position_cmd();
  }
  else if (control_command_.mode.id == ControlModeMsg::ID_MODE_SPEED)
  {
    actuator_->set_speed_command(control_command_.velocity);
    actuator_->send_speed_cmd();
  }

  actuator_->getState(actuator_state_);

  // Fill message with control mode for user feedback
  actuator_state_.mode = control_command_.mode;

  actuator_state_pub_->publish(actuator_state_);
  // RCLCPP_INFO(get_logger(), "ActuatorDriver::control_loop_cb_ end...");
}

void ActuatorDriver::control_command_cb_(const ControlCommandMsg::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(),
               "Control command message received (mode.id: %d | position: %f | velocity: %f | torque: %f)",
               msg->mode.id, msg->position, msg->velocity, msg->torque);
  control_command_ = *msg;
}

// TODO remove this service as we have a dedicated top now
void ActuatorDriver::set_control_mode_srv_cb_(
    const std::shared_ptr<actuator_msgs::srv::SetControlMode::Request> request,
    std::shared_ptr<actuator_msgs::srv::SetControlMode::Response> response)
{
  // RCLCPP_DEBUG(this->get_logger(), "Set control mode request received : %d", request->mode.id);
  // if ((request->mode.id == ControlModeMsg::ID_MODE_OFF) || (request->mode.id == ControlModeMsg::ID_MODE_TORQUE) ||
  //     (request->mode.id == ControlModeMsg::ID_MODE_POSITION))
  // {
  //   control_mode_.id = request->mode.id;
  //   response->success = true;
  // }
  // else
  // {
  //   response->success = false;
  // }
}

void ActuatorDriver::actuator_stop_()
{
  control_command_.mode.id = ControlModeMsg::ID_MODE_OFF;
  actuator_->stop();
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  std::shared_ptr<ActuatorDriver> node = std::make_shared<ActuatorDriver>("actuator_driver");
  exe.add_node(node->get_node_base_interface());
  RCLCPP_INFO(node->get_logger(), "Start spinning \'%s\'...", node->get_name());
  exe.spin();
  RCLCPP_INFO(node->get_logger(), "Shutting down \'%s\'...", node->get_name());
  rclcpp::shutdown();

  return 0;
}