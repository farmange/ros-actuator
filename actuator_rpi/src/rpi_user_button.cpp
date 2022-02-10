//============================================================================
// Name        : rpi_user_button.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "actuator_rpi/rpi_user_button.h"

using std::placeholders::_1;
using std::placeholders::_2;

RpiUserButton::RpiUserButton(rclcpp_lifecycle::LifecycleNode* node) : node_(node)
{
  led_blink_speed_ = 0;
  led_blink_counter_ = 0;
  led_blink_state_ = true;
  long_press_detected_ = false;
  press_counter_ = 0;

  /* Led is initialized in red color */
  set_rgb_led_(255, 0, 0, 0);

  init_parameters_();
  init_services_();

  /* Input buttons */
  pinMode(btn_up_pin_, INPUT);
  pinMode(btn_down_pin_, INPUT);
  pinMode(btn_mode_pin_, INPUT);
  pullUpDnControl(btn_up_pin_, PUD_UP);
  pullUpDnControl(btn_down_pin_, PUD_UP);
  pullUpDnControl(btn_mode_pin_, PUD_UP);

  /* Output LEDs */
  pinMode(led_red_pin_, OUTPUT);
  pinMode(led_green_pin_, OUTPUT);
  pinMode(led_blue_pin_, OUTPUT);

  if (softPwmCreate(led_red_pin_, red_led_state_, 100) != 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "led_red_pin_ softPwmCreate error !");
    return;
  }

  if (softPwmCreate(led_green_pin_, green_led_state_, 100) != 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "led_green_pin_ softPwmCreate error !");
    return;
  }

  if (softPwmCreate(led_blue_pin_, blue_led_state_, 100) != 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "led_blue_pin_ softPwmCreate error !");
    return;
  }
}

void RpiUserButton::update(bool& button_up, bool& button_down, bool& button_mode)
{
  /* Low side commutation */
  button_up = 1 - digitalRead(btn_up_pin_);
  button_down = 1 - digitalRead(btn_down_pin_);
  button_mode = 1 - digitalRead(btn_mode_pin_);
  process_button_mode_state_(button_mode);
  update_led_();
}

void RpiUserButton::init_parameters_()
{
  node_->declare_parameter<int>("rpi_user_btn_down_pin", 0);
  node_->declare_parameter<int>("rpi_user_btn_up_pin", 0);
  node_->declare_parameter<int>("rpi_user_btn_mode_pin", 0);
  node_->declare_parameter<int>("rpi_user_led_red_pin", 0);
  node_->declare_parameter<int>("rpi_user_led_green_pin", 0);
  node_->declare_parameter<int>("rpi_user_led_blue_pin", 0);
  node_->declare_parameter<double>("rpi_long_press_duration", 0.0);
  node_->declare_parameter<double>("rpi_inactivity_duration", 0.0);
  node_->get_parameter("rpi_user_btn_down_pin", btn_down_pin_);
  node_->get_parameter("rpi_user_btn_up_pin", btn_up_pin_);
  node_->get_parameter("rpi_user_btn_mode_pin", btn_mode_pin_);
  node_->get_parameter("rpi_user_led_red_pin", led_red_pin_);
  node_->get_parameter("rpi_user_led_green_pin", led_green_pin_);
  node_->get_parameter("rpi_user_led_blue_pin", led_blue_pin_);
  node_->get_parameter("rpi_long_press_duration", long_press_duration_);
  node_->get_parameter("rpi_inactivity_duration", inactivity_duration_);
}

void RpiUserButton::init_services_()
{
  button_mode_srv_client_ = node_->create_client<ButtonModeEventSrv>("button_mode_event");
  set_rgb_led_srv_server_ =
      node_->create_service<SetRgbLedSrv>("set_rgb_led", std::bind(&RpiUserButton::set_rgb_led_cb_, this, _1, _2));
}

void RpiUserButton::update_led_()
{
  if (led_blink_speed_ == 0)
  {
    led_blink_state_ = true;
  }
  else
  {
    if (led_blink_counter_ > led_blink_speed_)
    {
      led_blink_counter_ = 0;
      led_blink_state_ = !led_blink_state_;
    }
  }
  led_blink_counter_++;
  if (led_blink_state_)
  {
    /* Set led to user defined value */
    softPwmWrite(led_red_pin_, red_led_state_);
    softPwmWrite(led_green_pin_, green_led_state_);
    softPwmWrite(led_blue_pin_, blue_led_state_);
  }
  else
  {
    /* Set led to off */
    softPwmWrite(led_red_pin_, 100);
    softPwmWrite(led_green_pin_, 100);
    softPwmWrite(led_blue_pin_, 100);
  }
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

void RpiUserButton::set_rgb_led_(uint8_t r, uint8_t g, uint8_t b, uint8_t blink_speed)
{
  led_blink_speed_ = blink_speed;
  /* Warning : due to high side led wiring, output states are inverted
   * so that 0 mean 100% while 100 mean 0% .*/
  red_led_state_ = 100 - scale_rgb_pwm_(r);
  green_led_state_ = 100 - scale_rgb_pwm_(g);
  blue_led_state_ = 100 - scale_rgb_pwm_(b);
  /* Reset Blink counter when led color is set */
  led_blink_counter_ = 0;
  /* Force led state. This allow to set led state without calling updateLed_ method */
  softPwmWrite(led_red_pin_, red_led_state_);
  softPwmWrite(led_green_pin_, green_led_state_);
  softPwmWrite(led_blue_pin_, blue_led_state_);
}

void RpiUserButton::set_rgb_led_cb_(const std::shared_ptr<SetRgbLedSrv::Request> request,
                                    std::shared_ptr<SetRgbLedSrv::Response> response)
{
  RCLCPP_DEBUG(node_->get_logger(), "Set RGB LEDs with R:%d | G:%d | B:%d | blink_speed:%d", request->r, request->g,
               request->b, request->blink_speed);

  set_rgb_led_(request->r, request->g, request->b, request->blink_speed);
}

int RpiUserButton::scale_rgb_pwm_(uint8_t color)
{
  int result = ((int)color * 100) / 255;
  if (result > 100)
  {
    result = 100;
  }
  if (result < 0)
  {
    result = 0;
  }
  return result;
}