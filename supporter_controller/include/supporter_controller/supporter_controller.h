//============================================================================
// Name        : supporter_controller.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================
#ifndef SUPPORTER_CONTROLLER_H
#define SUPPORTER_CONTROLLER_H

// #include <memory>

#include "rclcpp/rclcpp.hpp"

#include "actuator_msgs/msg/control_command.hpp"
#include "actuator_msgs/msg/control_mode.hpp"
#include "actuator_msgs/msg/rpi_interface.hpp"
#include "actuator_msgs/msg/actuator_state.hpp"
#include "actuator_msgs/srv/button_event.hpp"
#include "actuator_msgs/srv/set_rgb_led.hpp"

// #include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "fsm/fsm.h"
#include "supporter_controller/fsm_user_input.h"

// #include "rclcpp_lifecycle/lifecycle_publisher.hpp"

// using rclcpp_lifecycle::node_interfaces;

class SupporterController : public rclcpp_lifecycle::LifecycleNode
{
  using ControlModeMsg = actuator_msgs::msg::ControlMode;
  using ControlCommandMsg = actuator_msgs::msg::ControlCommand;
  using RpiInterfaceMsg = actuator_msgs::msg::RpiInterface;
  using ActuatorStateMsg = actuator_msgs::msg::ActuatorState;
  using ButtonEventSrv = actuator_msgs::srv::ButtonEvent;
  using SetRgbLedSrv = actuator_msgs::srv::SetRgbLed;

public:
  SupporterController(const std::string& node_name, bool intra_process_comms = false);

  ~SupporterController();

  CallbackReturn on_configure(const rclcpp_lifecycle::State&);

  CallbackReturn on_activate(const rclcpp_lifecycle::State&);

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&);

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&);

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&);

private:
  rclcpp::Subscription<RpiInterfaceMsg>::SharedPtr rpi_interface_sub_;
  rclcpp::Subscription<ActuatorStateMsg>::SharedPtr actuator_state_sub_;
  rclcpp_lifecycle::LifecyclePublisher<ControlCommandMsg>::SharedPtr control_command_pub_;
  rclcpp::Service<ButtonEventSrv>::SharedPtr button_mode_event_srv_;
  rclcpp::Service<ButtonEventSrv>::SharedPtr button_updown_event_srv_;

  rclcpp::Client<SetRgbLedSrv>::SharedPtr set_rgb_led_srv_client_;

  rclcpp::TimerBase::SharedPtr timer_;

  RpiInterfaceMsg rpi_interface_;
  ActuatorStateMsg actuator_state_;
  ControlCommandMsg control_command_;
  ControlModeMsg current_control_mode_;

  bool new_control_command_received_;
  bool torque_preset_crossing_detected_;
  double torque_preset_param_;
  double save_default_mode_duration_param_;
  rclcpp::Time save_default_mode_start_time_;

  float high_velocity_param_;
  float low_velocity_param_;
  double vel_ramp_duration_param_;
  double low_velocity_duration_param_;
  bool adapt_vel_rising_edge_detected_;
  rclcpp::Time adapt_velocity_time_;

  double low_torque_inc_param_;
  double high_torque_inc_param_;
  double torque_ramp_duration_param_;
  double low_torque_duration_param_;
  bool adapt_torque_rising_edge_detected_;
  rclcpp::Time adapt_torque_time_;
  double torque_increment_;

  int loop_rate_param_;
  double short_vibration_amplitude_param_;
  double short_vibration_period_param_;
  double short_vibration_duration_param_;
  double long_vibration_amplitude_param_;
  double long_vibration_period_param_;
  double long_vibration_duration_param_;
  rclcpp::Time save_torque_start_time_;
  rclcpp::Time stall_torque_start_time_;

  ControlModeMsg default_control_mode_param_;

private:
  void init_parameters_();
  void init_publishers_();
  void init_subscribers_();
  void init_services_();
  void button_mode_event_cb_(const std::shared_ptr<ButtonEventSrv::Request> request,
                             std::shared_ptr<ButtonEventSrv::Response> response);
  void button_updown_event_cb_(const std::shared_ptr<ButtonEventSrv::Request> request,
                               std::shared_ptr<ButtonEventSrv::Response> response);
  void rpi_interface_cb_(const RpiInterfaceMsg::SharedPtr msg);
  void actuator_state_cb_(const ActuatorStateMsg::SharedPtr msg);

  void control_loop_cb_();
  void set_rgb_led_(uint8_t led_r, uint8_t led_g, uint8_t led_b, uint8_t led_blink_speed);
  void set_rgb_led_();
  void adapt_velocity_(float& velocity_cmd);
  void adapt_torque_(float& torque_cmd, const double& torque_increment);

  /* FSM engine */
  Engine<SupporterController>* engine_;

  /* FSM state */
  State<SupporterController>* state_uninitialized_;
  State<SupporterController>* state_stopped_;
  State<SupporterController>* state_speed_control_;
  State<SupporterController>* state_torque_control_;
  State<SupporterController>* state_save_torque_preset_;
  State<SupporterController>* state_stall_torque_preset_;
  State<SupporterController>* state_change_mode_;
  State<SupporterController>* state_save_default_mode_;

  /* FSM Transitions */
  Transition<SupporterController>* tr_to_speed_control_;
  Transition<SupporterController>* tr_to_torque_control_;
  Transition<SupporterController>* tr_to_stopped_;
  Transition<SupporterController>* tr_to_save_torque_preset_;
  Transition<SupporterController>* tr_to_stall_torque_preset_;
  Transition<SupporterController>* tr_exit_save_torque_preset_;
  Transition<SupporterController>* tr_exit_stall_torque_preset_;
  Transition<SupporterController>* tr_to_change_mode_;
  Transition<SupporterController>* tr_to_save_default_mode_;
  Transition<SupporterController>* tr_exit_save_default_mode_to_stopped_;
  Transition<SupporterController>* tr_exit_save_default_mode_to_speed_control_;
  Transition<SupporterController>* tr_exit_save_default_mode_to_torque_control_;

  bool tr_to_speed_control_cb_();
  bool tr_to_torque_control_cb_();
  bool tr_to_stopped_cb_();
  bool tr_to_save_torque_preset_cb_();
  bool tr_to_stall_torque_preset_cb_();
  bool tr_exit_save_torque_preset_cb_();
  bool tr_exit_stall_torque_preset_cb_();
  bool tr_to_change_mode_cb_();
  bool tr_to_save_default_mode_cb_();
  bool tr_exit_save_default_mode_to_stopped_cb_();
  bool tr_exit_save_default_mode_to_speed_control_cb_();
  bool tr_exit_save_default_mode_to_torque_control_cb_();

  /* FSM functions */
  void state_stopped_enter_();
  void state_stopped_update_();
  void state_speed_control_enter_();
  void state_speed_control_update_();
  void state_torque_control_enter_();
  void state_torque_control_update_();
  void state_save_torque_preset_enter_();
  void state_save_torque_preset_update_();
  void state_save_torque_preset_exit_();
  void state_stall_torque_preset_enter_();
  void state_stall_torque_preset_update_();
  void state_stall_torque_preset_exit_();
  void state_change_mode_enter_();
  void state_save_default_mode_enter_();

  /* FMS init function */
  void init_fsm_();

  FsmUserInput user_input_;
};
#endif