//============================================================================
// Name        : rpi_user_button.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : Class definition that bind all low level function
//               of the libkinovadrv
//============================================================================

#ifndef ACTUATOR_RPI_RPI_USER_BUTTON_H
#define ACTUATOR_RPI_RPI_USER_BUTTON_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "actuator_msgs/srv/button_event.hpp"
#include "actuator_msgs/srv/set_rgb_led.hpp"

#include <wiringPi.h>
#include <softPwm.h>

class RpiUserButton
{
  using ButtonEventSrv = actuator_msgs::srv::ButtonEvent;
  using SetRgbLedSrv = actuator_msgs::srv::SetRgbLed;

public:
  RpiUserButton(rclcpp_lifecycle::LifecycleNode* node);
  void update(bool& button_up, bool& button_down, bool& button_state);

private:
  rclcpp_lifecycle::LifecycleNode* node_;

  int loop_rate_;

  int btn_up_pin_;
  int btn_down_pin_;
  int btn_mode_pin_;
  int led_red_pin_;
  int led_green_pin_;
  int led_blue_pin_;

  void init_parameters_();
  void init_services_();
  void update_led_();

  void process_button_mode_state_(const bool& button_state);
  void process_button_updown_state_(const bool& button_up_state, const bool& button_down_state);
  void set_rgb_led_(uint8_t r, uint8_t g, uint8_t b, uint8_t blink_rate);
  void set_rgb_led_cb_(const std::shared_ptr<SetRgbLedSrv::Request> request,
                       std::shared_ptr<SetRgbLedSrv::Response> response);
  int scale_rgb_pwm_(uint8_t color);

  rclcpp::Client<ButtonEventSrv>::SharedPtr button_mode_srv_client_;
  rclcpp::Client<ButtonEventSrv>::SharedPtr button_updown_srv_client_;
  rclcpp::Service<SetRgbLedSrv>::SharedPtr set_rgb_led_srv_server_;

  /* RGB led handling attributes */
  uint8_t led_blink_rate_;
  uint32_t led_blink_counter_;
  bool led_blink_state_;
  uint8_t red_led_state_;
  uint8_t green_led_state_;
  uint8_t blue_led_state_;

  /* Button mode handling attributes */
  bool btn_mode_prev_state_;
  bool btn_mode_long_press_detected_;
  double btn_mode_long_press_duration_;
  double btn_mode_inactivity_duration_;
  rclcpp::Time btn_mode_press_time_;
  rclcpp::Time btn_mode_release_time_;
  int btn_mode_press_counter_;

  /* Button updown handling attributes */
  bool btn_up_prev_state_;
  bool btn_down_prev_state_;
  double btn_updown_long_press_duration_;
  double btn_updown_inactivity_duration_;
  rclcpp::Time btn_updown_press_time_;
  rclcpp::Time btn_updown_release_time_;
};

#endif
