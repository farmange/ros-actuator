#ifndef BASE_COMM_H
#define BASE_COMM_H

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class BaseComm
{
public:
  typedef enum CommStatus_e : uint8_t
  {
    COMM_STATUS_OK = 0,
    COMM_STATUS_INIT_ERR,
    COMM_STATUS_READ_ERR,
    COMM_STATUS_WRITE_ERR,
    COMM_STATUS_OTHER_ERR
  } CommStatus_t;

public:
  BaseComm(rclcpp_lifecycle::LifecycleNode* node) : node_(node){};
  // virtual ~BaseComm()
  // {
  // }

  virtual CommStatus_t init(const std::string& can_device, const uint8_t& reduction_ratio, const uint16_t& max_speed,
                            const int32_t& max_accel, const float& current_limit) = 0;
  // virtual int read() = 0;
  // virtual int write() = 0;

  /* Send position in degree */
  virtual CommStatus_t sendPosition(const float& joint_position_cmd) = 0;
  virtual CommStatus_t sendTorque(const float& joint_torque_cmd) = 0;

  virtual CommStatus_t startHardwareControlLoop() = 0;
  virtual CommStatus_t stopHardwareControlLoop() = 0;
  virtual CommStatus_t stop() = 0;
  virtual CommStatus_t start() = 0;

  virtual CommStatus_t getState(float& temperature, float& torque, float& ia, float& ib, float& ic, float& position,
                                float& speed, float& voltage, uint16_t& error) = 0;

protected:
  rclcpp_lifecycle::LifecycleNode* node_;

  uint8_t reduction_ratio_;  // 36
  float current_limit_;
  uint8_t pos_to_deg_factor_;  // 0.01 degree/LSB
  uint16_t max_speed_;         // 1080
  int32_t max_accel_;          // 700
};

#endif