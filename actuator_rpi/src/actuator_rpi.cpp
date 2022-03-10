//============================================================================
// Name        : actuator_rpi.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include <fstream>

#include "actuator_rpi/actuator_rpi.h"
#include "actuator_msgs/msg/rpi_interface.hpp"
#include <chrono>
using namespace std::chrono_literals;

ActuatorRpi::ActuatorRpi(const std::string& node_name, bool intra_process_comms)
  : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ActuatorRpi::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "ActuatorRpi::on_configure");

  init_parameters_();
  init_publishers_();

  /* These define the callback groups
   * They don't really do much on their own, but they have to exist in order to
   * assign callbacks to them. They're also what the executor looks for when trying to run multiple threads
   */
  callback_group_thread1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_thread2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  /* Each of these callback groups is basically a thread
   * Everything assigned to one of them gets bundled into the same thread
   */
  // auto thread1_opt = rclcpp::SubscriptionOptions();
  // thread1_opt.callback_group = callback_group_thread1_;
  // auto thread2_opt = rclcpp::SubscriptionOptions();
  // thread2_opt.callback_group = callback_group_thread2_;

  if (rpi_loop_rate_ <= 0)
  {
    RCLCPP_ERROR(get_logger(), "Wrong sampling frequency value (%d) for RPI thread", rpi_loop_rate_);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  if (wiringPiSetup() == -1)
  {
    RCLCPP_ERROR(get_logger(), "wiringPiSetup error !");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  // ROS_INFO("Create power button led");
  // power_button_led_.reset(new ArmmsPowerButtonLed(nh_));

  // ROS_INFO("Create user button");
  // user_button_.reset(new ArmmsUserButton(nh_));
  RCLCPP_INFO(get_logger(), "Create user button");
  user_button_ = std::make_shared<RpiUserButton>(this);

  // ROS_INFO("Create motor power");
  // motor_power_.reset(new ArmmsMotorPower(nh_));

  // ROS_INFO("Create switch limit");
  // switch_limit_.reset(new ArmmsSwitchLimit(nh_));
  RCLCPP_INFO(get_logger(), "Create switch limit");
  switch_limit_ = std::make_shared<RpiSwitchLimit>(this);

  // ROS_INFO("Create shutdown manager");
  // shutdown_manager_.reset(new ArmmsShutdownManager(nh_, power_button_led_.get()));

  // ROS_INFO("Create diagnostic thread");
  // rpi_diagnostics_.reset(new ArmmsDiagnostics(nh_));
  RCLCPP_INFO(get_logger(), "Create diagnostic thread");
  rpi_diagnostics_ = std::make_shared<RpiDiagnostics>(this, callback_group_thread1_);

  RCLCPP_INFO(get_logger(), "Initialize timer for rpi loop");
  std::chrono::duration<double> period(1.0 / rpi_loop_rate_);
  timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                   std::bind(&ActuatorRpi::update_, this), callback_group_thread2_);
  timer_->cancel();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ActuatorRpi::on_activate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "ActuatorRpi::on_activate");
  rpi_diagnostics_->start();
  rpi_interface_pub_->on_activate();
  timer_->reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ActuatorRpi::on_deactivate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "ActuatorRpi::on_deactivate");
  rpi_diagnostics_->stop();
  timer_->cancel();
  rpi_interface_pub_->on_deactivate();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ActuatorRpi::on_cleanup(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "ActuatorRpi::on_cleanup");
  rpi_diagnostics_->stop();
  timer_->cancel();
  rpi_interface_pub_->on_deactivate();
  rpi_diagnostics_.reset();
  timer_.reset();
  clear_publishers_();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ActuatorRpi::on_shutdown(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "ActuatorRpi::on_shutdown");
  rpi_diagnostics_->stop();
  timer_->cancel();
  rpi_interface_pub_->on_deactivate();
  rpi_diagnostics_.reset();
  timer_.reset();
  clear_publishers_();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void ActuatorRpi::init_parameters_()
{
  this->declare_parameter<int>("loop_rate", 100);
  this->get_parameter("loop_rate", rpi_loop_rate_);
}

void ActuatorRpi::init_publishers_()
{
  rpi_interface_pub_ = this->create_publisher<actuator_msgs::msg::RpiInterface>("rpi_interface", 1);
}

void ActuatorRpi::clear_publishers_()
{
  rpi_interface_pub_.reset();
}

void ActuatorRpi::update_()
{
  bool switch_limit;
  bool user_button_up;
  bool user_button_down;
  bool user_button_mode;

  user_button_->update(user_button_up, user_button_down, user_button_mode);
  switch_limit_->update(switch_limit);

  actuator_msgs::msg::RpiInterface msg;
  msg.switch_limit = switch_limit;
  msg.user_button_up = user_button_up;
  msg.user_button_down = user_button_down;
  msg.user_button_mode = user_button_mode;
  msg.cpu_temperature = rpi_diagnostics_->getCpuTemperature();
  msg.probe_temperature = rpi_diagnostics_->getProbeTemperature();
  rpi_interface_pub_->publish(msg);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exe;
  std::shared_ptr<ActuatorRpi> node = std::make_shared<ActuatorRpi>("actuator_rpi");
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();

  return 0;
}