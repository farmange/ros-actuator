//============================================================================
// Name        : support_controller_node.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "supporter_controller/supporter_controller.h"
#include <chrono>
// #include <unistd.h>
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

SupporterController::SupporterController(const std::string& node_name, bool intra_process_comms)
  : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
}

SupporterController::~SupporterController()
{
  RCLCPP_DEBUG(this->get_logger(), "Destructor of SupporterController");
  if (timer_ != nullptr)
  {
    timer_->cancel();
  }
}

SupporterController::CallbackReturn SupporterController::on_configure(const rclcpp_lifecycle::State&)
{
  init_parameters_();
  init_publishers_();
  init_subscribers_();
  init_services_();

  RCLCPP_INFO(get_logger(), "Initialize timer for supporter controller loop");
  timer_ = this->create_wall_timer(10ms, std::bind(&SupporterController::control_loop_cb_, this));
  timer_->cancel();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

SupporterController::CallbackReturn SupporterController::on_activate(const rclcpp_lifecycle::State&)
{
  enabled_ = false;
  control_command_pub_->on_activate();
  timer_->reset();
  control_command_.mode.id = ControlModeMsg::ID_MODE_OFF;
  control_mode_request_.id = ControlModeMsg::ID_MODE_SPEED;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

SupporterController::CallbackReturn SupporterController::on_deactivate(const rclcpp_lifecycle::State&)
{
  control_command_pub_->on_deactivate();
  timer_->cancel();
  control_command_.mode.id = ControlModeMsg::ID_MODE_OFF;
  control_mode_request_.id = ControlModeMsg::ID_MODE_SPEED;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

SupporterController::CallbackReturn SupporterController::on_cleanup(const rclcpp_lifecycle::State&)
{
  control_command_pub_.reset();
  timer_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

SupporterController::CallbackReturn SupporterController::on_shutdown(const rclcpp_lifecycle::State&)
{
  control_command_pub_->on_deactivate();
  timer_->cancel();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void SupporterController::init_parameters_()
{
  this->declare_parameter<double>("high_velocity", 10.0);
  this->declare_parameter<double>("low_velocity", 1.0);
  this->declare_parameter<double>("velocity_ramp_duration", 2.0);
  this->declare_parameter<double>("slow_velocity_duration", 5.0);

  this->declare_parameter<double>("low_torque_inc", 0.001);
  this->declare_parameter<double>("high_torque_inc", 0.01);
  this->declare_parameter<double>("torque_ramp_duration", 2.0);
  this->declare_parameter<double>("slow_torque_duration", 5.0);

  this->get_parameter("high_velocity", high_velocity_param_);
  this->get_parameter("low_velocity", low_velocity_param_);
  this->get_parameter("velocity_ramp_duration", vel_ramp_duration_param_);
  this->get_parameter("slow_velocity_duration", slow_velocity_duration_param_);

  this->get_parameter("low_torque_inc", low_torque_inc_param_);
  this->get_parameter("high_torque_inc", high_torque_inc_param_);
  this->get_parameter("torque_ramp_duration", torque_ramp_duration_param_);
  this->get_parameter("slow_torque_duration", slow_torque_duration_param_);
}

void SupporterController::init_publishers_()
{
  control_command_pub_ = this->create_publisher<ControlCommandMsg>("control_command", 1);
}

void SupporterController::init_subscribers_()
{
  rpi_interface_sub_ = this->create_subscription<RpiInterfaceMsg>(
      "rpi_interface", 1, std::bind(&SupporterController::rpi_interface_cb_, this, _1));
}

void SupporterController::init_services_()
{
  button_mode_event_srv_ = this->create_service<ButtonModeEventSrv>(
      "button_mode_event", std::bind(&SupporterController::button_mode_event_cb_, this, _1, _2));
}

void SupporterController::button_mode_event_cb_(const std::shared_ptr<ButtonModeEventSrv::Request> request,
                                                std::shared_ptr<ButtonModeEventSrv::Response> response)
{
  RCLCPP_DEBUG(this->get_logger(), "Receive button mode event : %d", request->id);

  if (request->id == ButtonModeEventSrv::Request::BTN_SHORT_PRESS)
  {
    if (control_mode_request_.id == ControlModeMsg::ID_MODE_OFF ||
        control_mode_request_.id == ControlModeMsg::ID_MODE_POSITION ||
        control_mode_request_.id == ControlModeMsg::ID_MODE_SPEED)
    {
      control_mode_request_.id = ControlModeMsg::ID_MODE_TORQUE;
    }
    else
    {
      control_mode_request_.id = ControlModeMsg::ID_MODE_SPEED;
    }
    enabled_ = false;
    response->success = true;
  }
  else if (request->id == ButtonModeEventSrv::Request::BTN_SHORT_DOUBLE_PRESS)
  {
    response->success = true;
  }
  else if (request->id == ButtonModeEventSrv::Request::BTN_LONG_PRESS)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void SupporterController::rpi_interface_cb_(const RpiInterfaceMsg::SharedPtr msg)
{
  rpi_interface_ = *msg;
  RCLCPP_WARN(get_logger(), "rpi_interface_.user_button_up: %d !", rpi_interface_.user_button_up);
  RCLCPP_WARN(get_logger(), "rpi_interface_.user_button_down: %d !", rpi_interface_.user_button_down);

  if (rpi_interface_.user_button_up != rpi_interface_.user_button_down)
  {
    enabled_ = true;
  }
}

void SupporterController::control_loop_cb_()
{
  /* Handle control mode */
  RCLCPP_WARN(get_logger(), "control_mode_request_.id: %d !", control_mode_request_.id);
  RCLCPP_WARN(get_logger(), "enable: %d !", enabled_);
  if ((control_mode_request_.id == ControlModeMsg::ID_MODE_SPEED) ||
      (control_mode_request_.id == ControlModeMsg::ID_MODE_TORQUE))
  {
    if (enabled_)
    {
      control_command_.mode = control_mode_request_;
      control_command_.torque = 0.0;

      /* Reset control mode request */
      control_mode_request_.id = ControlModeMsg::ID_MODE_OFF;
    }
    else
    {
      control_command_.mode.id = ControlModeMsg::ID_MODE_OFF;
    }
  }
  else if (control_mode_request_.id == ControlModeMsg::ID_MODE_OFF)
  {
    /* This mode is used to notify that no control mode change are requested
     * -> Keep the current control mode
     */
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Problem control mode handling: should never be there (requested mode: %d) !",
                 control_mode_request_.id);
    control_command_.mode.id = ControlModeMsg::ID_MODE_OFF;
  }

  RCLCPP_DEBUG(get_logger(), "Control command mode '%d'", control_command_.mode.id);

  /* Handle user button interface */
  control_command_.position = 0.0;
  control_command_.velocity = 0.0;

  if (control_command_.mode.id == ControlModeMsg::ID_MODE_SPEED)
  {
    if (rpi_interface_.user_button_up && !rpi_interface_.user_button_down)
    {
      /* UP*/
      control_command_.velocity = high_velocity_param_;
    }
    else if (!rpi_interface_.user_button_up && rpi_interface_.user_button_down)
    {
      /* DOWN */
      control_command_.velocity = -high_velocity_param_;
    }
    RCLCPP_DEBUG(get_logger(), "Control command init velocity '%f'", control_command_.velocity);
    double vel_adapted = control_command_.velocity;
    adapt_velocity_(vel_adapted);
    control_command_.velocity = vel_adapted;
  }
  else if (control_command_.mode.id == ControlModeMsg::ID_MODE_TORQUE)
  {
    if (rpi_interface_.user_button_up && !rpi_interface_.user_button_down)
    {
      /* UP*/
      if (control_command_.torque < 0.0)
      {
        /* Detect fast change of direction */
        control_command_.torque = 0.0;
      }
      control_command_.torque += low_torque_inc_param_;
    }
    else if (!rpi_interface_.user_button_up && rpi_interface_.user_button_down)
    {
      /* DOWN */
      if (control_command_.torque > 0.0)
      {
        /* Detect fast change of direction */
        control_command_.torque = 0.0;
      }
      control_command_.torque -= low_torque_inc_param_;
    }
    RCLCPP_DEBUG(get_logger(), "Control command init torque '%f'", control_command_.torque);
    double torque_adapted = control_command_.torque;
    // adapt_torque_(torque_adapted);
    control_command_.torque = torque_adapted;
  }
  else if (control_command_.mode.id == ControlModeMsg::ID_MODE_OFF)
  {
    control_command_.torque = 0.0;
  }
  else
  {
    control_command_.torque = 0.0;
    RCLCPP_ERROR(get_logger(), "Problem user button handling: should never be there (requested mode: %d) !",
                 control_command_.mode.id);
  }

  RCLCPP_DEBUG(get_logger(), "Control command torque '%f' and velocity '%f'", control_command_.torque,
               control_command_.velocity);

  // # Switch limit
  // bool switch_limit
  // # User button up
  // bool user_button_up
  // # User button down
  // bool user_button_down
  // # User button down
  // bool user_button_mode
  // # Raspberry pi temperature
  // float32 rpi_temperature

  control_command_pub_->publish(control_command_);
}

void SupporterController::adapt_velocity_(double& velocity_cmd)
{
  /* Detect rising edge */
  if (velocity_cmd != 0.0)
  {
    if (adapt_vel_rising_edge_detected_ == true)
    {
      adapt_velocity_time_ = get_clock()->now();
    }
    adapt_vel_rising_edge_detected_ = false;
  }
  else
  {
    adapt_vel_rising_edge_detected_ = true;
    return;
  }

  double step = 0.0;
  rclcpp::Duration started_since_duration = (get_clock()->now() - adapt_velocity_time_);
  rclcpp::Duration long_cmd_duration =
      started_since_duration - rclcpp::Duration::from_seconds(slow_velocity_duration_param_);

  /* If long command detected */
  if (long_cmd_duration > rclcpp::Duration::from_seconds(0.0))
  {
    /* while rampup duration is not exceed */
    if (long_cmd_duration < rclcpp::Duration::from_seconds(vel_ramp_duration_param_))
    {
      /* Compute the step to achieve to go from slow velocity to desired velocity */
      step = (high_velocity_param_ - low_velocity_param_);
      /* After slow_velocity_duration_param_, we start to linearly increase the velocity */
      step = step * (long_cmd_duration.seconds() / vel_ramp_duration_param_);
    }
    else
    {
      /* Do nothing (keep velocity_cmd as it is) */
      return;
    }
  }

  /* Update velocity setpoint according to the computed step and the velocity direction */
  if (velocity_cmd > 0.0)
  {
    velocity_cmd = low_velocity_param_ + step;
  }
  else
  {
    velocity_cmd = -low_velocity_param_ - step;
  }
}

double low_torque_inc_param_;
double high_torque_inc_param_;
double torque_ramp_duration_param_;
double slow_torque_duration_param_;

void SupporterController::adapt_torque_(double& torque_cmd)
{
  /* Detect rising edge */
  if (torque_cmd != 0.0)
  {
    if (adapt_torque_rising_edge_detected_ == true)
    {
      adapt_torque_time_ = get_clock()->now();
    }
    adapt_torque_rising_edge_detected_ = false;
  }
  else
  {
    adapt_torque_rising_edge_detected_ = true;
    return;
  }

  double step = 0.0;
  rclcpp::Duration started_since_duration = (get_clock()->now() - adapt_torque_time_);
  rclcpp::Duration long_cmd_duration =
      started_since_duration - rclcpp::Duration::from_seconds(slow_torque_duration_param_);

  /* If long command detected */
  if (long_cmd_duration > rclcpp::Duration::from_seconds(0.0))
  {
    /* while rampup duration is not exceed */
    if (long_cmd_duration < rclcpp::Duration::from_seconds(torque_ramp_duration_param_))
    {
      /* Compute the step to achieve to go from slow velocity to desired velocity */
      step = (high_torque_inc_param_ - low_torque_inc_param_);
      /* After slow_velocity_duration_param_, we start to linearly increase the velocity */
      step = step * (long_cmd_duration.seconds() / torque_ramp_duration_param_);
    }
    else
    {
      /* Do nothing (keep torque_cmd as it is) */
      return;
    }
  }

  /* Update velocity setpoint according to the computed step and the velocity direction */
  if (torque_cmd > 0.0)
  {
    torque_cmd = torque_cmd + (/* low_torque_inc_param_ */ +step);
  }
  else
  {
    torque_cmd = torque_cmd - (/* low_torque_inc_param_ */ +step);
  }
}
