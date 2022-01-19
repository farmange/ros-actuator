#ifndef BASE_COMM_H
#define BASE_COMM_H

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class BaseComm
{
public:
  BaseComm(rclcpp_lifecycle::LifecycleNode* node) : node_(node){};
  // virtual ~BaseComm()
  // {
  // }

  virtual int init(const std::string& can_device, const uint8_t& reduction_ratio, const uint16_t& max_speed,
                   const int32_t& max_accel, const float& current_limit) = 0;
  // virtual int read() = 0;
  // virtual int write() = 0;

  /* Send position in degree */
  virtual int sendPosition(const float& joint_position_cmd) = 0;
  virtual int sendTorque(const float& joint_torque_cmd) = 0;

  virtual void startHardwareControlLoop() = 0;
  virtual void stopHardwareControlLoop() = 0;
  virtual void stop() = 0;
  virtual void start() = 0;

  virtual int getState(float& temperature, float& torque, float& ia, float& ib, float& ic, float& position,
                       float& speed, float& voltage, std::string& error) = 0;

protected:
  rclcpp_lifecycle::LifecycleNode* node_;

  uint8_t reduction_ratio_;  // 36
  float current_limit_;
  uint8_t pos_to_deg_factor_;  // 0.01 degree/LSB
  uint16_t max_speed_;         // 1080
  int32_t max_accel_;          // 700
};

#endif