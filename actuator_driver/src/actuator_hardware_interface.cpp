
#include "actuator_driver/actuator_hardware_interface.h"

ActuatorHardwareInterface::ActuatorHardwareInterface(rclcpp_lifecycle::LifecycleNode* node,
                                                     std::shared_ptr<BaseComm> comm, const float& current_limit)
  : node_(node), comm_(comm), current_limit_(current_limit)
{
}

ActuatorHardwareInterface::~ActuatorHardwareInterface()
{
}

void ActuatorHardwareInterface::getState(actuator_msgs::msg::ActuatorState& actuator_state)
{
  actuator_state.torque_command = torque_command_;
  actuator_state.position_command = position_command_;
  actuator_state.temperature = temperature_;
  actuator_state.torque = torque_;
  actuator_state.ia = ia_;
  actuator_state.ib = ib_;
  actuator_state.ic = ic_;
  actuator_state.iq = iq_;
  actuator_state.position = position_;
  actuator_state.speed = speed_;
  actuator_state.voltage = voltage_;
  actuator_state.error = error_;

  return;
}

void ActuatorHardwareInterface::set_position_command(const float& position_command)
{
  position_command_ = position_command;
}

void ActuatorHardwareInterface::set_torque_command(const float& torque_command)
{
  torque_command_ = torque_command;
}

void ActuatorHardwareInterface::set_speed_command(const float& speed_command)
{
  speed_command_ = speed_command;
}

void ActuatorHardwareInterface::stop()
{
  comm_->stop();
}

void ActuatorHardwareInterface::start()
{
  comm_->start();
}

void ActuatorHardwareInterface::read()
{
  if (comm_->getState(temperature_, torque_, ia_, ib_, ic_, iq_, position_, speed_, voltage_, error_) ==
      BaseComm::COMM_STATUS_OK)
  {
    rclcpp::Clock ros_clock(RCL_ROS_TIME);

    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), ros_clock, 1000,
                          "Read actuator state \n"
                          "temp: %2.1f | torque: %2.3f | iq: %2.3f \n"
                          "ia: %2.3f | ib: %2.3f | ic: %2.3f | pos: %4.3f \n"
                          "vel: %3.2f | volt: %2.1f | err: %x",
                          temperature_, torque_, iq_, ia_, ib_, ic_, position_, speed_, voltage_, error_);
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "Cannot read actuator position !");
  }
  return;
}

void ActuatorHardwareInterface::send_position_cmd()
{
  if (comm_->sendPosition(position_command_) == BaseComm::COMM_STATUS_OK)
  {
    // RCLCPP_DEBUG(node_->get_logger(), "Write actuator position command: %f", position_command_);
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "Cannot write actuator position command %f !", position_command_);
    return;
  }
}

void ActuatorHardwareInterface::send_torque_cmd()
{
  if (comm_->sendTorque(torque_command_) == BaseComm::COMM_STATUS_OK)
  {
    // RCLCPP_DEBUG(node_->get_logger(), "Write actuator torque command: %f", torque_command_);
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "Cannot write actuator torque command %f !", torque_command_);
    return;
  }
}

void ActuatorHardwareInterface::send_speed_cmd()
{
  if (comm_->sendSpeed(speed_command_) == BaseComm::COMM_STATUS_OK)
  {
    // RCLCPP_DEBUG(node_->get_logger(), "Write actuator torque command: %f", torque_command_);
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "Cannot write actuator torque command %f !", torque_command_);
    return;
  }
}