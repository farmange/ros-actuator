
#include "actuator_driver/actuator_hardware_interface.h"

ActuatorHardwareInterface::ActuatorHardwareInterface(rclcpp::Node* node, BaseComm* comm) : node_(node), comm_(comm)
{
}

ActuatorHardwareInterface::~ActuatorHardwareInterface()
{
}

void ActuatorHardwareInterface::getState(actuator_msgs::msg::ActuatorState& actuator_state)
{
  actuator_state.position_command = position_command_;
  actuator_state.temperature = temperature_;
  actuator_state.iq = iq_;
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
  if (comm_->getState(temperature_, iq_, ia_, ib_, ic_, position_, speed_, voltage_, error_) == 0)
  {
    RCLCPP_INFO(node_->get_logger(),
                "Read actuator state \ntemp: %2.1f | iq: %2.3f | ia: %2.3f | ib: %2.3f | ic: %2.3f \npos: %4.3f | vel: "
                "%3.2f | volt: %2.1f | err: %s",
                temperature_, iq_, ia_, ib_, ic_, position_, speed_, voltage_, error_.c_str());
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "Cannot read actuator position !");
  }
  return;
}

void ActuatorHardwareInterface::write()
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