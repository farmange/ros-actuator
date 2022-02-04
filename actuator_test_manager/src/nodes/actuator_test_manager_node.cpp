//============================================================================
// Name        : actuator_driver.cpp
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
//============================================================================

#include "actuator_test_manager/nodes/actuator_test_manager_node.h"
#include <chrono>
// #include <unistd.h>
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

ActuatorTestManager::ActuatorTestManager(const std::string& node_name, bool intra_process_comms)
  : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
}

ActuatorTestManager::CallbackReturn ActuatorTestManager::on_configure(const rclcpp_lifecycle::State&)
{
  init_publishers_();
  init_subscribers_();
  init_services_();

  RCLCPP_INFO(get_logger(), "Initialize timer for test manager loop");
  timer_ = this->create_wall_timer(10ms, std::bind(&ActuatorTestManager::control_loop_cb_, this));
  timer_->cancel();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

ActuatorTestManager::~ActuatorTestManager()
{
  RCLCPP_DEBUG(this->get_logger(), "Destructor of ActuatorTestManager");
  if (timer_ != nullptr)
  {
    timer_->cancel();
  }
}

ActuatorTestManager::CallbackReturn ActuatorTestManager::on_activate(const rclcpp_lifecycle::State&)
{
  if (!set_control_mode_srvclient_->service_is_ready())
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  position_command_pub_->on_activate();
  torque_command_pub_->on_activate();
  timer_->reset();

  auto request = std::make_shared<actuator_msgs::srv::SetControlMode::Request>();
  request->mode.id = actuator_msgs::msgs::ControlMode::ID_MODE_POSITION;
  RCLCPP_DEBUG(this->get_logger(), "set_control_mode_srvclient_->async_send_request(request)");
  // TODO handle success false response (go back to inactive ?)
  auto result = set_control_mode_srvclient_->async_send_request(request);
  RCLCPP_DEBUG(this->get_logger(), "Done ");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

ActuatorTestManager::CallbackReturn ActuatorTestManager::on_deactivate(const rclcpp_lifecycle::State&)
{
  position_command_pub_->on_deactivate();
  torque_command_pub_->on_deactivate();
  timer_->cancel();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

ActuatorTestManager::CallbackReturn ActuatorTestManager::on_cleanup(const rclcpp_lifecycle::State&)
{
  position_command_pub_.reset();
  torque_command_pub_.reset();
  timer_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

ActuatorTestManager::CallbackReturn ActuatorTestManager::on_shutdown(const rclcpp_lifecycle::State&)
{
  position_command_pub_->on_deactivate();
  torque_command_pub_->on_deactivate();
  timer_->cancel();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void ActuatorTestManager::init_publishers_()
{
  position_command_pub_ = this->create_publisher<std_msgs::msg::Float32>("position_command", 1);
  torque_command_pub_ = this->create_publisher<std_msgs::msg::Float32>("torque_command", 1);
}

void ActuatorTestManager::init_subscribers_()
{
  actuator_state_sub_ = this->create_subscription<actuator_msgs::msg::ActuatorState>(
      "actuator_state", 1, std::bind(&ActuatorTestManager::actuator_state_cb_, this, _1));
}

void ActuatorTestManager::init_services_()
{
  set_control_mode_srvclient_ = this->create_client<actuator_msgs::srv::SetControlMode>("set_control_mode");
}

void ActuatorTestManager::actuator_state_cb_(const actuator_msgs::msg::ActuatorState::SharedPtr msg)
{
  actuator_state_ = *msg;
}

void ActuatorTestManager::control_loop_cb_()
{
  position_command_.data = 0.0;
  torque_command_.data = 0.0;

  position_command_pub_.publish(position_command_);
  torque_command_pub_.publish(torque_command_);

  // printf("Cycle N° %d / %d.\n", current_iteration + 1, desired_iteration);
  // printf("Elapsed time since beginning of experiment: %d min %d s\n", (uint16_t)((pause_currenttime - starttime)
  // / 60.),
  //        (uint16_t)(pause_currenttime - starttime) % 60);
  // int32_t desired_position = 0;

  // for (uint8_t i = 0; i < 2; i++)  // remplacer par un tableau à parcourir si plus
  // // de deux positions
  // {
  //   if (i == 0)
  //   {
  //     desired_position = desired_position_1;
  //   }
  //   else
  //   {
  //     desired_position = desired_position_2;
  //   }

  //   // Goto to desired position
  //   printf("Go to desired position [%d] : (%f) deg.\n", i, motorPosToDeg(desired_position));

  //   mc.positionControlCommand2(maxspeed, desired_position, temperature, iq, speed, encoder);
  //   printf("Windings temperature = %d°C\n", temperature);

  //   while ((current_position > (desired_position + position_error_tolerance)) ||
  //          (current_position < (desired_position - position_error_tolerance)))
  //   {
  //     // Read motor multi turn angle
  //     int64_t dummy_angle = 0;
  //     mc.readMultiTurnsAngleCommand(dummy_angle);
  //     current_position = static_cast<int32_t>(dummy_angle);

  //     std::vector<std::string> dataline(12, "");

  //     mc.readActuatorData(mc, desired_position, dataline, current_iteration);
  //     csv.writeLog(dataline);

  //     // Wait 50ms
  //     usleep(acquisition_period);
  //   }

  //   pause_starttime = getCurrentTime();
  //   printf("Starting pause of %d s.\n", (int8_t)pause_duration);

  //   while (pause_currenttime < pause_starttime + pause_duration)
  //   {
  //     std::vector<std::string> dataline(12, "");

  //     mc.readActuatorData(mc, desired_position, dataline, current_iteration);
  //     csv.writeLog(dataline);

  //     // Wait
  //     usleep(acquisition_period);
  //     pause_currenttime = getCurrentTime();
  //   }
  // }
  // current_iteration++;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  std::shared_ptr<ActuatorTestManager> node = std::make_shared<ActuatorTestManager>("actuator_test_manager");
  exe.add_node(node->get_node_base_interface());
  RCLCPP_INFO(node->get_logger(), "Start spinning \'%s\'...", node->get_name());
  exe.spin();
  RCLCPP_INFO(node->get_logger(), "Shutting down \'%s\'...", node->get_name());
  rclcpp::shutdown();

  return 0;
}