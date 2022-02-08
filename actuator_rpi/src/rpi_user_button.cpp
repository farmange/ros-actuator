//============================================================================
// Name        : rpi_user_button.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "actuator_rpi/rpi_user_button.h"

RpiUserButton::RpiUserButton(rclcpp_lifecycle::LifecycleNode* node) : node_(node)
{
  init_parameters_();
  init_services_();

  pinMode(btn_up_pin_, INPUT);
  pinMode(btn_down_pin_, INPUT);
  pinMode(btn_mode_pin_, INPUT);
  pullUpDnControl(btn_up_pin_, PUD_UP);
  pullUpDnControl(btn_down_pin_, PUD_UP);
  pullUpDnControl(btn_mode_pin_, PUD_UP);
}

void RpiUserButton::update(bool& button_up, bool& button_down, bool& button_mode)
{
  /* Low side commutation */
  button_up = 1 - digitalRead(btn_up_pin_);
  button_down = 1 - digitalRead(btn_down_pin_);
  button_mode = 1 - digitalRead(btn_mode_pin_);
  process_button_mode_state_(button_mode);
}

void RpiUserButton::init_parameters_()
{
  node_->declare_parameter<int>("rpi_user_btn_down_pin", 0);
  node_->declare_parameter<int>("rpi_user_btn_up_pin", 0);
  node_->declare_parameter<int>("rpi_user_btn_mode_pin", 0);
  node_->declare_parameter<double>("rpi_long_press_duration", 0.0);
  node_->declare_parameter<double>("rpi_inactivity_duration", 0.0);
  node_->get_parameter("rpi_user_btn_down_pin", btn_down_pin_);
  node_->get_parameter("rpi_user_btn_up_pin", btn_up_pin_);
  node_->get_parameter("rpi_user_btn_mode_pin", btn_mode_pin_);
  node_->get_parameter("rpi_long_press_duration", long_press_duration_);
  node_->get_parameter("rpi_inactivity_duration", inactivity_duration_);
}

void RpiUserButton::init_services_()
{
  button_mode_srv_client_ = node_->create_client<ButtonModeEventSrv>("button_mode_event");
}

void RpiUserButton::process_button_mode_state_(const bool& button_mode)
{
  auto request = std::make_shared<ButtonModeEventSrv::Request>();
  request->id = ButtonModeEventSrv::Request::BTN_NONE;

  if (button_mode)
  {
    if (!button_mode_prev_)
    {
      /* Rising edge of button */
      press_time_ = node_->get_clock()->now();
    }
    /* Raise event if button is maintained during long time */
    if ((node_->get_clock()->now() - press_time_) > rclcpp::Duration::from_seconds(long_press_duration_))
    {
      /* Long press detected */
      request->id = ButtonModeEventSrv::Request::BTN_LONG_PRESS;
      press_time_ = node_->get_clock()->now();
      long_press_detected_ = true;
    }
  }
  else
  {
    /* Falling edge detected */
    if (button_mode_prev_)
    {
      if (long_press_detected_ != true)
      {
        /* short press detected */
        press_counter_++;
        release_time_ = node_->get_clock()->now();
      }
      else
      {
        /* If falling edge of a long press : do not take into account this event */
        long_press_detected_ = false;
      }
    }
    if (press_counter_ > 0)
    {
      /* After a specified inactivity delay, raise button event according to number of press detected */
      if ((node_->get_clock()->now() - release_time_) > rclcpp::Duration::from_seconds(inactivity_duration_))
      {
        if (press_counter_ == 1)
        {
          request->id = ButtonModeEventSrv::Request::BTN_SHORT_PRESS;
        }
        else if (press_counter_ == 2)
        {
          request->id = ButtonModeEventSrv::Request::BTN_SHORT_DOUBLE_PRESS;
        }
        else
        {
          press_counter_ = 0;
        }
      }
    }
  }

  /* Only send service request if an event is detected */
  if (request->id != ButtonModeEventSrv::Request::BTN_NONE)
  {
    press_counter_ = 0;
    if (button_mode_srv_client_->service_is_ready())
    {
      button_mode_srv_client_->async_send_request(request);
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "No service server found for button event");
    }
  }

  button_mode_prev_ = button_mode;
}
