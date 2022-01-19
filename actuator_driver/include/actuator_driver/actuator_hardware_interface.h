// #include <linux/can.h>
// #include <linux/can/raw.h>
// #include <sys/socket.h>
// #include <vector>
// #include <string.h>
// #include <fstream>
// #include <stdio.h>
// #include <sys/types.h>

#ifndef ACTUATOR_HARDWARE_INTERFACE_H
#define ACTUATOR_HARDWARE_INTERFACE_H

#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "actuator_driver/base_comm.h"
#include "actuator_msgs/msg/actuator_state.hpp"

class ActuatorHardwareInterface
{
public:
  typedef enum status_e
  {
    OK = 0,
    READ_ERROR,
    WRITE_ERROR
  } status_t;
  ActuatorHardwareInterface(rclcpp_lifecycle::LifecycleNode* node, std::shared_ptr<BaseComm> comm);
  ~ActuatorHardwareInterface();
  void getState(actuator_msgs::msg::ActuatorState& actuator_state);
  void setPositionCommand(const float& position_command);
  void setTorqueCommand(const float& torque_command);

  void stop();
  void start();

  // void init();
  // void initPosition();
  // void update(const ros::TimerEvent& e);
  // void setCommandToCurrentPosition();
  void read();
  //   void enforceLimit(ros::Duration elapsed_time);
  //   void resetLimit();
  void send_position_cmd();
  void send_torque_cmd();
  //   status_t getStatus();
private:
  std::shared_ptr<BaseComm> comm_;
  rclcpp_lifecycle::LifecycleNode* node_;

  float position_command_;
  float torque_command_;
  float temperature_;
  float torque_;
  float ia_;
  float ib_;
  float ic_;
  float position_;
  float speed_;
  float voltage_;
  uint16_t error_;
};

#endif