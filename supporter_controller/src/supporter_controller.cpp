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

  if (loop_rate_param_ <= 0)
  {
    RCLCPP_ERROR(get_logger(), "Wrong sampling frequency value (%d) for controller thread", loop_rate_param_);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Initialize timer for supporter controller loop");
  std::chrono::duration<double> period(1.0 / loop_rate_param_);
  timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                   std::bind(&SupporterController::control_loop_cb_, this));
  timer_->cancel();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

SupporterController::CallbackReturn SupporterController::on_activate(const rclcpp_lifecycle::State&)
{
  enabled_ = false;
  adapt_vel_rising_edge_detected_ = false;
  adapt_torque_rising_edge_detected_ = false;
  /* Dummy initialisation of ros time */
  adapt_velocity_time_ = get_clock()->now();
  adapt_torque_time_ = get_clock()->now();
  torque_increment_ = 0.0;

  // Test vibration
  control_command_.position = 4;

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
  this->declare_parameter<double>("low_velocity_duration", 5.0);

  this->declare_parameter<double>("low_torque_inc", 0.001);
  this->declare_parameter<double>("high_torque_inc", 0.01);
  this->declare_parameter<double>("torque_ramp_duration", 2.0);
  this->declare_parameter<double>("low_torque_duration", 5.0);

  this->declare_parameter<int>("loop_rate", 100);

  this->get_parameter("high_velocity", high_velocity_param_);
  this->get_parameter("low_velocity", low_velocity_param_);
  this->get_parameter("velocity_ramp_duration", vel_ramp_duration_param_);
  this->get_parameter("low_velocity_duration", low_velocity_duration_param_);

  this->get_parameter("low_torque_inc", low_torque_inc_param_);
  this->get_parameter("high_torque_inc", high_torque_inc_param_);
  this->get_parameter("torque_ramp_duration", torque_ramp_duration_param_);
  this->get_parameter("low_torque_duration", low_torque_duration_param_);

  this->get_parameter("loop_rate", loop_rate_param_);

  RCLCPP_DEBUG(get_logger(), "Initialize parameters with following value...");
  RCLCPP_DEBUG(get_logger(), "low_torque_inc: %f", low_torque_inc_param_);
  RCLCPP_DEBUG(get_logger(), "high_torque_inc: %f", high_torque_inc_param_);
  RCLCPP_DEBUG(get_logger(), "torque_ramp_duration: %f", torque_ramp_duration_param_);
  RCLCPP_DEBUG(get_logger(), "low_torque_duration: %f", low_torque_duration_param_);
  RCLCPP_DEBUG(get_logger(), "loop_rate : %d", loop_rate_param_);
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
  set_rgb_led_srv_client_ = this->create_client<SetRgbLedSrv>("set_rgb_led");
}

void SupporterController::button_mode_event_cb_(const std::shared_ptr<ButtonModeEventSrv::Request> request,
                                                std::shared_ptr<ButtonModeEventSrv::Response> response)
{
  RCLCPP_DEBUG(this->get_logger(), "Receive button mode event : %d", request->id);

  if (request->id == ButtonModeEventSrv::Request::BTN_SHORT_PRESS)
  {
    if (control_command_.mode.id == ControlModeMsg::ID_MODE_SPEED ||
        (control_command_.mode.id == ControlModeMsg::ID_MODE_OFF &&
         control_mode_request_.id == ControlModeMsg::ID_MODE_SPEED))
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

  if (rpi_interface_.user_button_up != rpi_interface_.user_button_down)
  {
    enabled_ = true;
  }
}

void SupporterController::control_loop_cb_()
{
  // Test vibration
  control_command_.mode.id = ControlModeMsg::ID_MODE_POSITION;
  control_command_.position = -control_command_.position;
  control_command_pub_->publish(control_command_);

  return;

  /*** Handle control mode ***/
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

  RCLCPP_DEBUG(get_logger(), "Current control command mode: %d (requested: %d, enabled: %d)", control_command_.mode.id,
               control_mode_request_.id, enabled_);

  /*** Handle user button interface ***/
  control_command_.position = 0.0;
  control_command_.velocity = 0.0;

  if (control_command_.mode.id == ControlModeMsg::ID_MODE_SPEED)
  {
    if (rpi_interface_.user_button_up && !rpi_interface_.user_button_down)
    {
      /* UP*/
      control_command_.velocity = low_velocity_param_;
      RCLCPP_ERROR(get_logger(), "UP velocity:%f", control_command_.velocity);
    }
    else if (!rpi_interface_.user_button_up && rpi_interface_.user_button_down)
    {
      /* DOWN */
      control_command_.velocity = -low_velocity_param_;
      RCLCPP_ERROR(get_logger(), "DOWN velocity:%f", control_command_.velocity);
    }
    torque_increment_ = 0.0;
    control_command_.torque = 0.0;
  }
  else if (control_command_.mode.id == ControlModeMsg::ID_MODE_TORQUE)
  {
    if (rpi_interface_.user_button_up && !rpi_interface_.user_button_down)
    {
      /* UP*/
      torque_increment_ = low_torque_inc_param_;
      RCLCPP_ERROR(get_logger(), "UP torque_increment_:%f", torque_increment_);
    }
    else if (!rpi_interface_.user_button_up && rpi_interface_.user_button_down)
    {
      /* DOWN */
      torque_increment_ = -low_torque_inc_param_;
      RCLCPP_ERROR(get_logger(), "DOWN torque_increment_:%f", torque_increment_);
    }
    else
    {
      torque_increment_ = 0.0;
    }
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
  adapt_velocity_(control_command_.velocity);
  adapt_torque_(control_command_.torque, torque_increment_);

  RCLCPP_DEBUG(get_logger(), "Control command torque '%f' and velocity '%f'", control_command_.torque,
               control_command_.velocity);

  control_command_pub_->publish(control_command_);

  /* Update LED according to control_mode_request_ and enabled_ attributes */
  set_rgb_led_();
}

void SupporterController::set_rgb_led_()

{
  uint8_t led_r, led_g, led_b, led_blink_speed = 0;
  if (control_mode_request_.id == ControlModeMsg::ID_MODE_SPEED)
  {
    /* BLINK BLUE : mode speed inactive */
    led_r = 0;
    led_g = 0;
    led_b = 255;
    led_blink_speed = 10;
  }
  else if (control_mode_request_.id == ControlModeMsg::ID_MODE_TORQUE)
  {
    /* BLINK GREEN : mode torque inactive */
    led_r = 0;
    led_g = 255;
    led_b = 0;
    led_blink_speed = 10;
  }
  else if (control_command_.mode.id == ControlModeMsg::ID_MODE_SPEED)
  {
    /* SOLID BLUE : mode speed active */
    led_r = 0;
    led_g = 0;
    led_b = 255;
    led_blink_speed = 0;
  }

  else if (control_command_.mode.id == ControlModeMsg::ID_MODE_TORQUE)
  {
    /* SOLID GREEN : mode torque active */
    led_r = 0;
    led_g = 255;
    led_b = 0;
    led_blink_speed = 0;
  }
  else
  {
    /* RED : anything else (e.g. error) */
    led_r = 255;
    led_g = 0;
    led_b = 0;
  }

  auto request = std::make_shared<SetRgbLedSrv::Request>();
  request->r = led_r;
  request->g = led_g;
  request->b = led_b;
  request->blink_speed = led_blink_speed;

  if (set_rgb_led_srv_client_->service_is_ready())
  {
    set_rgb_led_srv_client_->async_send_request(request);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "No service server found to set rgb led");
  }
}

void SupporterController::adapt_velocity_(float& velocity_cmd)
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

  float step = 0.0;
  rclcpp::Duration started_since_duration = (get_clock()->now() - adapt_velocity_time_);
  rclcpp::Duration long_cmd_duration =
      started_since_duration - rclcpp::Duration::from_seconds(low_velocity_duration_param_);

  /* If long command detected */
  if (long_cmd_duration > rclcpp::Duration::from_seconds(0.0))
  {
    /* Compute the step to achieve to go from slow velocity to desired velocity */
    step = (high_velocity_param_ - low_velocity_param_);

    /* while rampup duration is not exceed */
    if (long_cmd_duration < rclcpp::Duration::from_seconds(vel_ramp_duration_param_))
    {
      /* After low_velocity_duration_param_, we start to linearly increase the velocity */
      step = step * (long_cmd_duration.seconds() / vel_ramp_duration_param_);
    }
    RCLCPP_DEBUG(get_logger(), "Linearly increase velocity command adding %f dps", step);
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

void SupporterController::adapt_torque_(float& torque_cmd, const double& torque_increment)
{
  /* Detect rising edge */
  if (torque_increment != 0.0)
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
      started_since_duration - rclcpp::Duration::from_seconds(low_torque_duration_param_);

  /* If long command detected */
  if (long_cmd_duration > rclcpp::Duration::from_seconds(0.0))
  {
    /* Compute the step to achieve to go from low torque to high torque */
    step = (high_torque_inc_param_ - low_torque_inc_param_);

    /* while rampup duration is not exceed */
    if (long_cmd_duration < rclcpp::Duration::from_seconds(torque_ramp_duration_param_))
    {
      /* After low_velocity_duration_param_, we start to linearly increase the velocity */
      step = step * (long_cmd_duration.seconds() / torque_ramp_duration_param_);
    }
    RCLCPP_DEBUG(get_logger(), "Linearly increase torque command adding %f Nm/0.1s", step);
  }

  /* Update velocity setpoint according to the computed step and the velocity direction */
  if (torque_increment > 0.0)
  {
    torque_cmd = torque_cmd + torque_increment + step;
  }
  else if (torque_increment < 0.0)
  {
    torque_cmd = torque_cmd + torque_increment - step;
  }
}
