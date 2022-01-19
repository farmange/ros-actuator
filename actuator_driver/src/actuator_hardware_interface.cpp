
#include "actuator_driver/actuator_hardware_interface.h"

ActuatorHardwareInterface::ActuatorHardwareInterface(rclcpp_lifecycle::LifecycleNode* node,
                                                     std::shared_ptr<BaseComm> comm)
  : node_(node), comm_(comm)
{
}

ActuatorHardwareInterface::~ActuatorHardwareInterface()
{
}

void ActuatorHardwareInterface::getState(actuator_msgs::msg::ActuatorState& actuator_state)
{
  actuator_state.position_command = position_command_;
  actuator_state.temperature = temperature_;
  actuator_state.torque = torque_;
  actuator_state.ia = ia_;
  actuator_state.ib = ib_;
  actuator_state.ic = ic_;
  actuator_state.position = position_;
  actuator_state.speed = speed_;
  actuator_state.voltage = voltage_;
  actuator_state.error = error_.c_str();
  return;
}

void ActuatorHardwareInterface::setPositionCommand(const float& position_command)
{
  position_command_ = position_command;
}

void ActuatorHardwareInterface::setTorqueCommand(const float& torque_command)
{
  torque_command_ = torque_command;
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
  if (comm_->getState(temperature_, torque_, ia_, ib_, ic_, position_, speed_, voltage_, error_) == 0)
  {
    RCLCPP_INFO(node_->get_logger(),
                "Read actuator state \n"
                "temp: %2.1f | torque: %2.3f | ia: %2.3f \n"
                "ib: %2.3f | ic: %2.3f | pos: %4.3f \n"
                "vel: %3.2f | volt: %2.1f | err: %s",
                temperature_, torque_, ia_, ib_, ic_, position_, speed_, voltage_, error_.c_str());
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "Cannot read actuator position !");
  }
  return;
}

void ActuatorHardwareInterface::send_position_cmd()
{
  if (comm_->sendPosition(position_command_) == 0)
  {
    RCLCPP_INFO(node_->get_logger(), "Write actuator position command: %f", position_command_);
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "Cannot write actuator position command %f !", position_command_);
    return;
  }
}

void ActuatorHardwareInterface::send_torque_cmd()
{
  if (comm_->sendTorque(torque_command_) == 0)
  {
    RCLCPP_INFO(node_->get_logger(), "Write actuator torque command: %f", torque_command_);
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "Cannot write actuator torque command %f !", torque_command_);
    return;
  }
}