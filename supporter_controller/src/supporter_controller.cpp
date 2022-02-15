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
  init_fsm_();

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
  adapt_vel_rising_edge_detected_ = false;
  adapt_torque_rising_edge_detected_ = false;
  /* Dummy initialisation of ros time */
  adapt_velocity_time_ = get_clock()->now();
  adapt_torque_time_ = get_clock()->now();
  torque_increment_ = 0.0;

  user_input_ = FsmUserInput::None;

  control_command_pub_->on_activate();
  timer_->reset();
  control_command_.mode.id = ControlModeMsg::ID_MODE_OFF;

  current_control_mode_ = default_control_mode_param_;
  new_control_command_received_ = false;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

SupporterController::CallbackReturn SupporterController::on_deactivate(const rclcpp_lifecycle::State&)
{
  control_command_pub_->on_deactivate();
  timer_->cancel();
  control_command_.mode.id = ControlModeMsg::ID_MODE_OFF;

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

  this->declare_parameter<double>("low_torque_inc", 0.01);
  this->declare_parameter<double>("high_torque_inc", 0.02);
  this->declare_parameter<double>("torque_ramp_duration", 2.0);
  this->declare_parameter<double>("low_torque_duration", 5.0);

  this->declare_parameter<int>("loop_rate", 100);
  this->declare_parameter<double>("short_vibration_amplitude", 0.1);
  this->declare_parameter<double>("short_vibration_period", 0.1);
  this->declare_parameter<double>("short_vibration_duration", 0.2);
  this->declare_parameter<double>("long_vibration_amplitude", 0.1);
  this->declare_parameter<double>("long_vibration_period", 0.1);
  this->declare_parameter<double>("long_vibration_duration", 0.2);
  this->declare_parameter<double>("torque_preset", 1.0);
  this->declare_parameter<int>("default_control_mode", 1);
  this->declare_parameter<double>("save_default_mode_duration", 1.0);

  this->get_parameter("high_velocity", high_velocity_param_);
  this->get_parameter("low_velocity", low_velocity_param_);
  this->get_parameter("velocity_ramp_duration", vel_ramp_duration_param_);
  this->get_parameter("low_velocity_duration", low_velocity_duration_param_);

  this->get_parameter("low_torque_inc", low_torque_inc_param_);
  this->get_parameter("high_torque_inc", high_torque_inc_param_);
  this->get_parameter("torque_ramp_duration", torque_ramp_duration_param_);
  this->get_parameter("low_torque_duration", low_torque_duration_param_);

  this->get_parameter("loop_rate", loop_rate_param_);
  this->get_parameter("short_vibration_amplitude", short_vibration_amplitude_param_);
  this->get_parameter("short_vibration_period", short_vibration_period_param_);
  this->get_parameter("short_vibration_duration", short_vibration_duration_param_);
  this->get_parameter("long_vibration_amplitude", long_vibration_amplitude_param_);
  this->get_parameter("long_vibration_period", long_vibration_period_param_);
  this->get_parameter("long_vibration_duration", long_vibration_duration_param_);
  this->get_parameter("torque_preset", torque_preset_param_);
  this->get_parameter("default_control_mode", default_control_mode_param_.id);
  this->get_parameter("save_default_mode_duration", save_default_mode_duration_param_);

  RCLCPP_DEBUG(get_logger(), "Initialize parameters with following value...");
  RCLCPP_DEBUG(get_logger(), "high_velocity: %f", high_velocity_param_);
  RCLCPP_DEBUG(get_logger(), "low_velocity: %f", low_velocity_param_);
  RCLCPP_DEBUG(get_logger(), "velocity_ramp_duration: %f", vel_ramp_duration_param_);
  RCLCPP_DEBUG(get_logger(), "low_velocity_duration: %f", low_velocity_duration_param_);

  RCLCPP_DEBUG(get_logger(), "low_torque_inc: %f", low_torque_inc_param_);
  RCLCPP_DEBUG(get_logger(), "high_torque_inc: %f", high_torque_inc_param_);
  RCLCPP_DEBUG(get_logger(), "torque_ramp_duration: %f", torque_ramp_duration_param_);
  RCLCPP_DEBUG(get_logger(), "low_torque_duration: %f", low_torque_duration_param_);

  RCLCPP_DEBUG(get_logger(), "loop_rate : %d", loop_rate_param_);
  RCLCPP_DEBUG(get_logger(), "short_vibration_amplitude : %f", short_vibration_amplitude_param_);
  RCLCPP_DEBUG(get_logger(), "short_vibration_period : %f", short_vibration_period_param_);
  RCLCPP_DEBUG(get_logger(), "short_vibration_duration : %f", short_vibration_duration_param_);
  RCLCPP_DEBUG(get_logger(), "long_vibration_amplitude : %f", long_vibration_amplitude_param_);
  RCLCPP_DEBUG(get_logger(), "long_vibration_period : %f", long_vibration_period_param_);
  RCLCPP_DEBUG(get_logger(), "long_vibration_duration : %f", long_vibration_duration_param_);
  RCLCPP_DEBUG(get_logger(), "torque_preset : %f", torque_preset_param_);
  RCLCPP_DEBUG(get_logger(), "default_control_mode : %d (1: torque | 3: speed)", default_control_mode_param_.id);
  RCLCPP_DEBUG(get_logger(), "save_default_mode_duration : %f", save_default_mode_duration_param_);
}

void SupporterController::init_publishers_()
{
  control_command_pub_ = this->create_publisher<ControlCommandMsg>("control_command", 1);
}

void SupporterController::init_subscribers_()
{
  rpi_interface_sub_ = this->create_subscription<RpiInterfaceMsg>(
      "rpi_interface", 1, std::bind(&SupporterController::rpi_interface_cb_, this, _1));
  actuator_state_sub_ = this->create_subscription<ActuatorStateMsg>(
      "actuator_state", 1, std::bind(&SupporterController::actuator_state_cb_, this, _1));
}

void SupporterController::init_services_()
{
  button_mode_event_srv_ = this->create_service<ButtonEventSrv>(
      "button_mode_event", std::bind(&SupporterController::button_mode_event_cb_, this, _1, _2));
  button_updown_event_srv_ = this->create_service<ButtonEventSrv>(
      "button_updown_event", std::bind(&SupporterController::button_updown_event_cb_, this, _1, _2));
  set_rgb_led_srv_client_ = this->create_client<SetRgbLedSrv>("set_rgb_led");
}

void SupporterController::button_mode_event_cb_(const std::shared_ptr<ButtonEventSrv::Request> request,
                                                std::shared_ptr<ButtonEventSrv::Response> response)
{
  RCLCPP_DEBUG(this->get_logger(), "Receive button mode event : %d", request->id);

  if (request->id == ButtonEventSrv::Request::BTN_SHORT_PRESS)
  {
    user_input_ = FsmUserInput::ButtonModeShortPress;
    response->success = true;
  }
  else if (request->id == ButtonEventSrv::Request::BTN_SHORT_DOUBLE_PRESS)
  {
    user_input_ = FsmUserInput::ButtonModeDoublePress;

    response->success = true;
  }
  else if (request->id == ButtonEventSrv::Request::BTN_LONG_PRESS)
  {
    user_input_ = FsmUserInput::ButtonModeLongPress;
    response->success = true;
  }
  else
  {
    user_input_ = FsmUserInput::None;
    response->success = false;
  }
}

void SupporterController::button_updown_event_cb_(const std::shared_ptr<ButtonEventSrv::Request> request,
                                                  std::shared_ptr<ButtonEventSrv::Response> response)
{
  RCLCPP_DEBUG(this->get_logger(), "Receive button updown event : %d", request->id);

  if (request->id == ButtonEventSrv::Request::BTN_LONG_PRESS)
  {
    user_input_ = FsmUserInput::ButtonUpDownLongPress;
    response->success = true;
  }
  else
  {
    user_input_ = FsmUserInput::None;
    response->success = false;
  }
}

void SupporterController::rpi_interface_cb_(const RpiInterfaceMsg::SharedPtr msg)
{
  rpi_interface_ = *msg;

  if (engine_->getCurrentState() == state_stopped_ || engine_->getCurrentState() == state_stall_torque_preset_)
  {
    if (rpi_interface_.user_button_up != rpi_interface_.user_button_down)
    {
      new_control_command_received_ = true;
    }
  }
}

void SupporterController::actuator_state_cb_(const ActuatorStateMsg::SharedPtr msg)
{
  actuator_state_ = *msg;
}

void SupporterController::control_loop_cb_()
{
  if ((current_control_mode_.id != ControlModeMsg::ID_MODE_SPEED) &&
      (current_control_mode_.id != ControlModeMsg::ID_MODE_TORQUE))
  {
    RCLCPP_ERROR(this->get_logger(), "Current control mode not supported !");
    return;
  }

  RCLCPP_WARN(get_logger(), "Current state is '%s' and input event is '%s'",
              engine_->getCurrentState()->getName().c_str(), user_input_.toString().c_str());
  engine_->process();

  user_input_ = FsmUserInput::None;

  return;
}

void SupporterController::set_rgb_led_(uint8_t led_r, uint8_t led_g, uint8_t led_b, uint8_t led_blink_speed)
{
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

void SupporterController::set_rgb_led_()
{
  // uint8_t led_r, led_g, led_b, led_blink_speed = 0;
  // if (control_mode_request_.id == ControlModeMsg::ID_MODE_SPEED)
  // {
  //   /* BLINK BLUE : mode speed inactive */
  //   led_r = 0;
  //   led_g = 0;
  //   led_b = 255;
  //   led_blink_speed = 10;
  // }
  // else if (control_mode_request_.id == ControlModeMsg::ID_MODE_TORQUE)
  // {
  //   /* BLINK GREEN : mode torque inactive */
  //   led_r = 0;
  //   led_g = 255;
  //   led_b = 0;
  //   led_blink_speed = 10;
  // }
  // else if (control_command_.mode.id == ControlModeMsg::ID_MODE_SPEED)
  // {
  //   /* SOLID BLUE : mode speed active */
  //   led_r = 0;
  //   led_g = 0;
  //   led_b = 255;
  //   led_blink_speed = 0;
  // }

  // else if (control_command_.mode.id == ControlModeMsg::ID_MODE_TORQUE)
  // {
  //   /* SOLID GREEN : mode torque active */
  //   led_r = 0;
  //   led_g = 255;
  //   led_b = 0;
  //   led_blink_speed = 0;
  // }
  // else
  // {
  //   /* RED : anything else (e.g. error) */
  //   led_r = 255;
  //   led_g = 0;
  //   led_b = 0;
  // }

  // auto request = std::make_shared<SetRgbLedSrv::Request>();
  // request->r = led_r;
  // request->g = led_g;
  // request->b = led_b;
  // request->blink_speed = led_blink_speed;

  // if (set_rgb_led_srv_client_->service_is_ready())
  // {
  //   set_rgb_led_srv_client_->async_send_request(request);
  // }
  // else
  // {
  //   RCLCPP_WARN(this->get_logger(), "No service server found to set rgb led");
  // }
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
  }

  /* Update velocity setpoint according to the computed step and the velocity direction */
  if (velocity_cmd > 0.0)
  {
    RCLCPP_DEBUG(get_logger(), "Linearly increase velocity command adding %f dps", step);
    velocity_cmd = low_velocity_param_ + step;
  }
  else
  {
    RCLCPP_DEBUG(get_logger(), "Linearly decrease velocity command removing %f dps", step);
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
    step = (high_torque_inc_param_ - low_torque_inc_param_) / (loop_rate_param_ * 1.0);

    /* while rampup duration is not exceed */
    if (long_cmd_duration < rclcpp::Duration::from_seconds(torque_ramp_duration_param_))
    {
      /* After low_velocity_duration_param_, we start to linearly increase the velocity */
      step = step * (long_cmd_duration.seconds() / torque_ramp_duration_param_);
    }
  }

  /* Update velocity setpoint according to the computed step and the velocity direction */
  if (torque_increment > 0.0)
  {
    RCLCPP_DEBUG(get_logger(), "Linearly increase torque command adding %f Nm/s", step);
    torque_cmd = torque_cmd + torque_increment + step;
  }
  else if (torque_increment < 0.0)
  {
    RCLCPP_DEBUG(get_logger(), "Linearly decrease torque command removing %f Nm/s", step);
    torque_cmd = torque_cmd + torque_increment - step;
  }
}

void SupporterController::init_fsm_()
{
  /* Engine definition */
  engine_ = new Engine<SupporterController>(this);

  /* State definition */
  state_uninitialized_ = new State<SupporterController>(this, engine_, "UNINITIALIZED");

  state_stopped_ = new State<SupporterController>(this, engine_, "STOPPED");
  state_stopped_->registerEnterFcn(&SupporterController::state_stopped_enter_);
  state_stopped_->registerUpdateFcn(&SupporterController::state_stopped_update_);

  state_speed_control_ = new State<SupporterController>(this, engine_, "SPEED CONTROL");
  state_speed_control_->registerEnterFcn(&SupporterController::state_speed_control_enter_);
  state_speed_control_->registerUpdateFcn(&SupporterController::state_speed_control_update_);

  state_torque_control_ = new State<SupporterController>(this, engine_, "TORQUE CONTROL");
  state_torque_control_->registerEnterFcn(&SupporterController::state_torque_control_enter_);
  state_torque_control_->registerUpdateFcn(&SupporterController::state_torque_control_update_);

  state_save_torque_preset_ = new State<SupporterController>(this, engine_, "SAVE TORQUE PRESET");
  state_save_torque_preset_->registerEnterFcn(&SupporterController::state_save_torque_preset_enter_);
  state_save_torque_preset_->registerUpdateFcn(&SupporterController::state_save_torque_preset_update_);
  state_save_torque_preset_->registerExitFcn(&SupporterController::state_save_torque_preset_exit_);

  state_stall_torque_preset_ = new State<SupporterController>(this, engine_, "STALL TORQUE PRESET");
  state_stall_torque_preset_->registerEnterFcn(&SupporterController::state_stall_torque_preset_enter_);
  state_stall_torque_preset_->registerUpdateFcn(&SupporterController::state_stall_torque_preset_update_);
  state_stall_torque_preset_->registerExitFcn(&SupporterController::state_stall_torque_preset_exit_);

  state_change_mode_ = new State<SupporterController>(this, engine_, "CHANGE MODE");
  state_change_mode_->registerEnterFcn(&SupporterController::state_change_mode_enter_);

  state_save_default_mode_ = new State<SupporterController>(this, engine_, "SAVE DEFAULT MODE");
  state_save_default_mode_->registerEnterFcn(&SupporterController::state_save_default_mode_enter_);

  /* Transitions definition */
  tr_to_speed_control_ = new Transition<SupporterController>(this, engine_, state_speed_control_);
  tr_to_speed_control_->registerConditionFcn(&SupporterController::tr_to_speed_control_cb_);
  tr_to_speed_control_->addInitialState(state_stopped_);

  tr_to_torque_control_ = new Transition<SupporterController>(this, engine_, state_torque_control_);
  tr_to_torque_control_->registerConditionFcn(&SupporterController::tr_to_torque_control_cb_);
  tr_to_torque_control_->addInitialState(state_stopped_);

  tr_to_stopped_ = new Transition<SupporterController>(this, engine_, state_stopped_);
  tr_to_stopped_->registerConditionFcn(&SupporterController::tr_to_stopped_cb_);
  tr_to_stopped_->addInitialState(state_uninitialized_);
  tr_to_stopped_->addInitialState(state_speed_control_);
  tr_to_stopped_->addInitialState(state_torque_control_);
  tr_to_stopped_->addInitialState(state_change_mode_);

  tr_to_save_torque_preset_ = new Transition<SupporterController>(this, engine_, state_save_torque_preset_);
  tr_to_save_torque_preset_->registerConditionFcn(&SupporterController::tr_to_save_torque_preset_cb_);
  tr_to_save_torque_preset_->addInitialState(state_torque_control_);

  tr_exit_save_torque_preset_ = new Transition<SupporterController>(this, engine_, state_torque_control_);
  tr_exit_save_torque_preset_->registerConditionFcn(&SupporterController::tr_exit_save_torque_preset_cb_);
  tr_exit_save_torque_preset_->addInitialState(state_save_torque_preset_);

  tr_to_stall_torque_preset_ = new Transition<SupporterController>(this, engine_, state_stall_torque_preset_);
  tr_to_stall_torque_preset_->registerConditionFcn(&SupporterController::tr_to_stall_torque_preset_cb_);
  tr_to_stall_torque_preset_->addInitialState(state_torque_control_);

  tr_exit_stall_torque_preset_ = new Transition<SupporterController>(this, engine_, state_torque_control_);
  tr_exit_stall_torque_preset_->registerConditionFcn(&SupporterController::tr_exit_stall_torque_preset_cb_);
  tr_exit_stall_torque_preset_->addInitialState(state_stall_torque_preset_);

  tr_to_change_mode_ = new Transition<SupporterController>(this, engine_, state_change_mode_);
  tr_to_change_mode_->registerConditionFcn(&SupporterController::tr_to_change_mode_cb_);
  tr_to_change_mode_->addInitialState(state_speed_control_);
  tr_to_change_mode_->addInitialState(state_torque_control_);
  tr_to_change_mode_->addInitialState(state_stopped_);

  tr_to_save_default_mode_ = new Transition<SupporterController>(this, engine_, state_save_default_mode_);
  tr_to_save_default_mode_->registerConditionFcn(&SupporterController::tr_to_save_default_mode_cb_);
  tr_to_save_default_mode_->addInitialState(state_speed_control_);
  tr_to_save_default_mode_->addInitialState(state_torque_control_);
  tr_to_save_default_mode_->addInitialState(state_stopped_);

  tr_exit_save_default_mode_to_stopped_ = new Transition<SupporterController>(this, engine_, state_stopped_);
  tr_exit_save_default_mode_to_stopped_->registerConditionFcn(
      &SupporterController::tr_exit_save_default_mode_to_stopped_cb_);
  tr_exit_save_default_mode_to_stopped_->addInitialState(state_save_default_mode_);

  tr_exit_save_default_mode_to_speed_control_ =
      new Transition<SupporterController>(this, engine_, state_speed_control_);
  tr_exit_save_default_mode_to_speed_control_->registerConditionFcn(
      &SupporterController::tr_exit_save_default_mode_to_speed_control_cb_);
  tr_exit_save_default_mode_to_speed_control_->addInitialState(state_save_default_mode_);

  tr_exit_save_default_mode_to_torque_control_ =
      new Transition<SupporterController>(this, engine_, state_torque_control_);
  tr_exit_save_default_mode_to_torque_control_->registerConditionFcn(
      &SupporterController::tr_exit_save_default_mode_to_torque_control_cb_);
  tr_exit_save_default_mode_to_torque_control_->addInitialState(state_save_default_mode_);

  engine_->setCurrentState(state_uninitialized_);
}

/*** FSM TRANSITION ***/
bool SupporterController::tr_to_speed_control_cb_()
{
  return ((current_control_mode_.id == ControlModeMsg::ID_MODE_SPEED) && new_control_command_received_);
}

bool SupporterController::tr_to_torque_control_cb_()
{
  return ((current_control_mode_.id == ControlModeMsg::ID_MODE_TORQUE) && new_control_command_received_);
}

bool SupporterController::tr_to_stopped_cb_()
{
  return ((user_input_ == FsmUserInput::ButtonModeDoublePress) || (engine_->getCurrentState() == state_change_mode_) ||
          (engine_->getCurrentState() == state_uninitialized_));
}

bool SupporterController::tr_to_save_torque_preset_cb_()
{
  return (user_input_ == FsmUserInput::ButtonUpDownLongPress);
}

bool SupporterController::tr_exit_save_torque_preset_cb_()
{
  return ((get_clock()->now() - save_torque_start_time_) >
          rclcpp::Duration::from_seconds(long_vibration_duration_param_));
  ;
}

bool SupporterController::tr_to_stall_torque_preset_cb_()
{
  return torque_preset_crossing_detected_;
}

bool SupporterController::tr_exit_stall_torque_preset_cb_()
{
  return ((get_clock()->now() - stall_torque_start_time_) >
          rclcpp::Duration::from_seconds(short_vibration_duration_param_));
  ;
}

bool SupporterController::tr_to_change_mode_cb_()
{
  return (user_input_ == FsmUserInput::ButtonModeShortPress);
}

bool SupporterController::tr_to_save_default_mode_cb_()
{
  return (user_input_ == FsmUserInput::ButtonModeLongPress);
}

bool SupporterController::tr_exit_save_default_mode_to_stopped_cb_()
{
  return ((control_command_.mode.id == ControlModeMsg::ID_MODE_OFF) &&
          ((get_clock()->now() - save_default_mode_start_time_) >
           rclcpp::Duration::from_seconds(save_default_mode_duration_param_)));
}

bool SupporterController::tr_exit_save_default_mode_to_speed_control_cb_()
{
  return ((control_command_.mode.id == ControlModeMsg::ID_MODE_SPEED) &&
          ((get_clock()->now() - save_default_mode_start_time_) >
           rclcpp::Duration::from_seconds(save_default_mode_duration_param_)));
}

bool SupporterController::tr_exit_save_default_mode_to_torque_control_cb_()
{
  return ((control_command_.mode.id == ControlModeMsg::ID_MODE_TORQUE) &&
          ((get_clock()->now() - save_default_mode_start_time_) >
           rclcpp::Duration::from_seconds(save_default_mode_duration_param_)));
}

/*** FSM STATE ***/
void SupporterController::state_stopped_enter_()
{
  new_control_command_received_ = false;
  if (current_control_mode_.id == ControlModeMsg::ID_MODE_SPEED)
  {
    set_rgb_led_(0, 0, 255, 50);
  }
  else if (current_control_mode_.id == ControlModeMsg::ID_MODE_TORQUE)
  {
    set_rgb_led_(0, 255, 0, 50);
  }
}

void SupporterController::state_stopped_update_()
{
  control_command_.mode.id = ControlModeMsg::ID_MODE_OFF;
  control_command_.position = 0.0;
  control_command_.velocity = 0.0;
  control_command_.torque = 0.0;
  control_command_pub_->publish(control_command_);
}

void SupporterController::state_speed_control_enter_()
{
  /* Do stuff to ensure adapt_velocity works as expected */
  control_command_.velocity = 0.0;
  adapt_velocity_(control_command_.velocity);
  set_rgb_led_(0, 0, 255, 0);
}

void SupporterController::state_speed_control_update_()
{
  control_command_.mode.id = ControlModeMsg::ID_MODE_SPEED;
  control_command_.position = 0.0;
  control_command_.velocity = 0.0;
  control_command_.torque = 0.0;

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

  adapt_velocity_(control_command_.velocity);

  RCLCPP_DEBUG(get_logger(), "Control velocity command '%f'", control_command_.velocity);

  control_command_pub_->publish(control_command_);
}

void SupporterController::state_torque_control_enter_()
{
  torque_preset_crossing_detected_ = false;
  float dummy_torque = 0.0;
  double dummy_increment = 0.0;
  adapt_torque_(dummy_torque, dummy_increment);
  set_rgb_led_(0, 255, 0, 0);
}

void SupporterController::state_torque_control_update_()
{
  control_command_.mode.id = ControlModeMsg::ID_MODE_TORQUE;
  control_command_.position = 0.0;
  control_command_.velocity = 0.0;

  if (rpi_interface_.user_button_up && !rpi_interface_.user_button_down)
  {
    /* UP*/
    torque_increment_ = low_torque_inc_param_ / (loop_rate_param_ * 1.0);
    RCLCPP_ERROR(get_logger(), "UP torque_increment_:%f", torque_increment_);
  }
  else if (!rpi_interface_.user_button_up && rpi_interface_.user_button_down)
  {
    /* DOWN */
    torque_increment_ = -low_torque_inc_param_ / (loop_rate_param_ * 1.0);
    RCLCPP_ERROR(get_logger(), "DOWN torque_increment_:%f", torque_increment_);
  }
  else
  {
    torque_increment_ = 0;
  }
  float torque_adapted = control_command_.torque;
  adapt_torque_(torque_adapted, torque_increment_);

  if (torque_preset_param_ != 0 && control_command_.torque != torque_preset_param_)
  {
    if (((control_command_.torque < torque_preset_param_) && (torque_adapted > torque_preset_param_)) ||
        ((control_command_.torque > torque_preset_param_) && (torque_adapted < torque_preset_param_)))
    {
      torque_preset_crossing_detected_ = true;
      torque_adapted = torque_preset_param_;
    }
  }
  else
  {
    torque_preset_crossing_detected_ = false;
  }
  control_command_.torque = torque_adapted;

  RCLCPP_DEBUG(get_logger(), "Control torque command '%f'", control_command_.torque);

  control_command_pub_->publish(control_command_);
}

/********************************/
void SupporterController::state_save_torque_preset_enter_()
{
  RCLCPP_DEBUG(get_logger(), "Save torque command '%f' as the new torque preset", control_command_.torque);
  new_control_command_received_ = false;
  torque_preset_param_ = control_command_.torque;
  save_torque_start_time_ = get_clock()->now();
}

void SupporterController::state_save_torque_preset_update_()
{
  /* long vibration*/
}

void SupporterController::state_save_torque_preset_exit_()
{
}

/********************************/
void SupporterController::state_stall_torque_preset_enter_()
{
  new_control_command_received_ = false;
  stall_torque_start_time_ = get_clock()->now();

  control_command_.mode.id = ControlModeMsg::ID_MODE_POSITION;
  control_command_.position = actuator_state_.position + short_vibration_amplitude_param_;
  control_command_pub_->publish(control_command_);
}

void SupporterController::state_stall_torque_preset_update_()
{
  /* short vibration */
  control_command_.mode.id = ControlModeMsg::ID_MODE_POSITION;

  rclcpp::Duration duration = (get_clock()->now() - stall_torque_start_time_);
  if (fmod(abs(duration.seconds()), short_vibration_duration_param_) < (short_vibration_duration_param_ / 2.0))
  {
    control_command_.position = actuator_state_.position + short_vibration_amplitude_param_;
  }
  else
  {
    control_command_.position = actuator_state_.position - short_vibration_amplitude_param_;
  }
  control_command_pub_->publish(control_command_);
}

void SupporterController::state_stall_torque_preset_exit_()
{
  /* Go back to init position */
  control_command_.mode.id = ControlModeMsg::ID_MODE_POSITION;
  control_command_.position = actuator_state_.position;
  control_command_pub_->publish(control_command_);

  /* Restore torque mode */
  control_command_.mode.id = ControlModeMsg::ID_MODE_TORQUE;
  torque_preset_crossing_detected_ = false;
}

/********************************/
void SupporterController::state_change_mode_enter_()
{
  if (current_control_mode_.id == ControlModeMsg::ID_MODE_SPEED)
  {
    current_control_mode_.id = ControlModeMsg::ID_MODE_TORQUE;
  }
  else if (current_control_mode_.id == ControlModeMsg::ID_MODE_TORQUE)
  {
    current_control_mode_.id = ControlModeMsg::ID_MODE_SPEED;
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Current_control_mode = %d not supported !", current_control_mode_.id);
    current_control_mode_.id = ControlModeMsg::ID_MODE_SPEED;
  }
}

void SupporterController::state_save_default_mode_enter_()
{
  RCLCPP_DEBUG(get_logger(), "Save '%d' as the new default mode", current_control_mode_.id);
  default_control_mode_param_ = current_control_mode_;
  if (current_control_mode_.id == ControlModeMsg::ID_MODE_SPEED)
  {
    set_rgb_led_(0, 0, 255, 10);
  }
  else if (current_control_mode_.id == ControlModeMsg::ID_MODE_TORQUE)
  {
    set_rgb_led_(0, 255, 0, 10);
  }
  save_default_mode_start_time_ = get_clock()->now();
}
