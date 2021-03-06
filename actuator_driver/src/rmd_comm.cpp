#include <cstdio>
// #include <stdlib.h>
// #include <iostream>
#include <string>
// #include <unistd.h>
// #include <net/if.h>
// #include <sys/ioctl.h>
// #include <linux/sockios.h>
// #include <sys/types.h>
#include "rclcpp/rclcpp.hpp"

#include "actuator_driver/rmd_comm.h"

const uint8_t RMDComm::ANGLE_TO_0_01DEG = 100;
const uint8_t RMDComm::SPEED_TO_0_01DPS = 100;
const float RMDComm::TORQUE_CONSTANT = 6.92;  // Nm/A (given in datasheet for gearbox output side)

RMDComm::RMDComm(rclcpp_lifecycle::LifecycleNode* node) : BaseComm(node)
{
}

RMDComm::CommStatus_t RMDComm::init(const std::string& can_device, const uint8_t& reduction_ratio,
                                    const uint16_t& max_speed, const int32_t& max_accel, const float& current_limit)
{
  max_speed_ = max_speed;  // 1dps/LSB, 1080/36 = 30 dps (same as partner param);
  max_accel_ = max_accel;  // 1dps² / LSB
  reduction_ratio_ = reduction_ratio;
  current_limit_ = current_limit;  // Amperes
  CanDriver::DriverStatus_t ret = can_interface_.init(can_device);
  if (ret != CanDriver::DRIVER_STATUS_OK)
  {
    return RMDComm::COMM_STATUS_INIT_ERR;
  }
  return writeAccelData2Ram_(max_accel_);
}

RMDComm::CommStatus_t RMDComm::sendPosition(const float& joint_position_cmd)
{
  int32_t motor_position = conv_joint_deg_to_motor_pos_(joint_position_cmd);
  RCLCPP_DEBUG(node_->get_logger(), "RMDComm: Send position command %f° (raw motor position: %d)", joint_position_cmd,
               motor_position);
  int8_t dummy_temperature;
  int16_t dummy_iq;
  int16_t dummy_speed;
  uint16_t dummy_encoder;
  return positionControlCommand2_(max_speed_, motor_position, dummy_temperature, dummy_iq, dummy_speed, dummy_encoder);
}

RMDComm::CommStatus_t RMDComm::sendTorque(const float& joint_torque_cmd)
{
  int32_t iq_command = conv_joint_torque_to_iq_(joint_torque_cmd);
  RCLCPP_DEBUG(node_->get_logger(), "RMDComm: Send torque command %f Nm (raw iq current: %d)", joint_torque_cmd,
               iq_command);
  int8_t dummy_temperature;
  int16_t dummy_iq;
  int16_t dummy_speed;
  uint16_t dummy_encoder;
  return torqueCurrentControlCommand_(iq_command, dummy_temperature, dummy_iq, dummy_speed, dummy_encoder);
}

RMDComm::CommStatus_t RMDComm::sendSpeed(const float& joint_speed_cmd)
{
  int32_t speed_command = conv_joint_speed_to_motor_speed_(joint_speed_cmd);
  RCLCPP_DEBUG(node_->get_logger(), "RMDComm: Send speed command %f dps (raw motor speed: %d)", joint_speed_cmd,
               speed_command);
  int8_t dummy_temperature;
  int16_t dummy_iq;
  int16_t dummy_speed;
  uint16_t dummy_encoder;
  return speedControlCommand_(speed_command, dummy_temperature, dummy_iq, dummy_speed, dummy_encoder);
}

RMDComm::CommStatus_t RMDComm::stop()
{
  return motorOffCommand_();
}

RMDComm::CommStatus_t RMDComm::start()
{
  return motorRunningCommand_();
}

RMDComm::CommStatus_t RMDComm::getState(float& temperature, float& torque, float& ia, float& ib, float& ic, float& iq,
                                        float& position, float& speed, float& voltage, uint16_t& error)
{
  int8_t raw_temperature;
  uint16_t raw_errorstate;
  int16_t raw_iq, raw_speed;
  int16_t raw_ia, raw_ib, raw_ic;
  uint16_t raw_voltage;
  uint16_t raw_encoder;
  int32_t raw_current_position = 0;
  RMDComm::CommStatus_t ret;

  temperature = 0.0;
  torque = 0.0;
  ia = 0.0;
  ib = 0.0;
  ic = 0.0;
  iq = 0.0;
  position = 0.0;
  speed = 0.0;
  voltage = 0.0;
  error = 0;

  // Read motor multi turn angle
  int64_t dummy_angle = 0;
  ret = readMultiTurnsAngleCommand_(dummy_angle);
  if (ret != RMDComm::COMM_STATUS_OK)
  {
    RCLCPP_ERROR(node_->get_logger(), "readMultiTurnsAngleCommand_ failed !");
    return ret;
  }
  raw_current_position = static_cast<int32_t>(dummy_angle);

  // Read motor error status, voltage and temperature
  ret = readMotorStatus1_(raw_temperature, raw_voltage, raw_errorstate);
  if (ret != RMDComm::COMM_STATUS_OK)
  {
    RCLCPP_ERROR(node_->get_logger(), "readMotorStatus1_ failed !");
    return ret;
  }

  // Read motor temperature, iq, speed and encoder
  ret = readMotorStatus2_(raw_temperature, raw_iq, raw_speed, raw_encoder);
  if (ret != RMDComm::COMM_STATUS_OK)
  {
    RCLCPP_ERROR(node_->get_logger(), "readMotorStatus2_ failed !");
    return ret;
  }

  // Read 3 phases current
  ret = readMotorStatus3_(raw_ia, raw_ib, raw_ic);
  if (ret != RMDComm::COMM_STATUS_OK)
  {
    RCLCPP_ERROR(node_->get_logger(), "readMotorStatus3_ failed !");
    return ret;
  }

  temperature = raw_temperature;
  torque = conv_iq_to_joint_torque_(raw_iq);
  ia = conv_iabc_to_ampere_(raw_ia);
  ib = conv_iabc_to_ampere_(raw_ib);
  ic = conv_iabc_to_ampere_(raw_ic);
  iq = conv_iq_to_ampere_(raw_iq);
  position = conv_motor_pos_to_joint_deg_(raw_current_position);
  speed = conv_motor_speed_to_joint_dps_(raw_speed);
  voltage = conv_voltage_to_volt_(raw_voltage);
  error = raw_errorstate;

  // TODO actuator error handling
  // if (raw_errorstate != 0)
  // {
  //   return 1;
  // }

  return RMDComm::COMM_STATUS_OK;
}

/***************************************************/
// Return the motor position in deg from the joint position
int32_t RMDComm::conv_joint_deg_to_motor_pos_(const double& joint_deg) const
{
  return static_cast<int32_t>(joint_deg * reduction_ratio_ * ANGLE_TO_0_01DEG);
}

// Return the joint position in deg from the motor position
double RMDComm::conv_motor_pos_to_joint_deg_(const int32_t& motor_pos) const
{
  return ((static_cast<double>(motor_pos)) / (reduction_ratio_ * ANGLE_TO_0_01DEG));
}

// Return the motor speed in 0.01dps from the joint position in deg
int32_t RMDComm::conv_joint_speed_to_motor_speed_(const double& joint_dps) const
{
  return static_cast<int32_t>(joint_dps * reduction_ratio_ * SPEED_TO_0_01DPS);
}

int16_t RMDComm::conv_joint_torque_to_iq_(const float& torque) const
{
  return (conv_ampere_to_iq_(torque / (TORQUE_CONSTANT)));
}
float RMDComm::conv_iq_to_joint_torque_(const int16_t& iq) const
{
  return conv_iq_to_ampere_(iq) * TORQUE_CONSTANT;
}

float RMDComm::conv_iq_to_ampere_(const int16_t& iq) const
{
  return static_cast<float>((static_cast<float>(iq) * 33.) / 2048.);
}

int16_t RMDComm::conv_ampere_to_iq_(const float& ampere) const
{
  return static_cast<int16_t>((ampere * 2000.) / 32.);
}

float RMDComm::conv_iabc_to_ampere_(const int16_t& iabc) const
{
  return (static_cast<float>(iabc) / 64.);
}

float RMDComm::conv_motor_speed_to_joint_dps_(const int16_t& motor_speed) const
{
  return (static_cast<float>(motor_speed) / reduction_ratio_);
}

float RMDComm::conv_voltage_to_volt_(const uint16_t& voltage) const
{
  return (static_cast<float>(voltage) / 10.);
}

int16_t RMDComm::saturate_current_limit_(const int16_t& iq) const
{
  if (iq > conv_ampere_to_iq_(current_limit_))
  {
    RCLCPP_WARN(node_->get_logger(), "RMDComm: Current limited to %f A (raw iq current: %d)", current_limit_,
                conv_ampere_to_iq_(current_limit_));

    return conv_ampere_to_iq_(current_limit_);
  }
  else
  {
    if (iq < conv_ampere_to_iq_(-current_limit_))
    {
      RCLCPP_WARN(node_->get_logger(), "RMDComm: Current limited to %f A (raw iq current: %d)", current_limit_,
                  conv_ampere_to_iq_(current_limit_));
      return conv_ampere_to_iq_(-current_limit_);
    }
  }
  return iq;
}

RMDComm::CommStatus_t RMDComm::send_receive_(const CanDriver::SendFrame_t& send_frame,
                                             CanDriver::ReceiveFrame_t& receive_frame)
{
  CanDriver::DriverStatus_t ret = can_interface_.sendReceive(send_frame, receive_frame);

  if (ret != CanDriver::DRIVER_STATUS_OK)
  {
    if (ret == CanDriver::DRIVER_STATUS_READ_ERR)
    {
      return COMM_STATUS_READ_ERR;
    }
    else if (ret == CanDriver::DRIVER_STATUS_WRITE_ERR)
    {
      return COMM_STATUS_WRITE_ERR;
    }
    else
    {
      return COMM_STATUS_OTHER_ERR;
    }
  }
  else
  {
    return COMM_STATUS_OK;
  }
}

/******************************************/
RMDComm::CommStatus_t RMDComm::readMotorStatus1_(int8_t& temperature, uint16_t& voltage, uint16_t& errorstate)
{
  CanDriver::SendFrame_t sendframe;
  CanDriver::ReceiveFrame_t receiveframe;

  sendframe.frame.can_id = 0x140 + 0x1;
  sendframe.frame.can_dlc = 8;
  sendframe.frame.data[0] = 0x9A;
  sendframe.frame.data[1] = 0;
  sendframe.frame.data[2] = 0;
  sendframe.frame.data[3] = 0;
  sendframe.frame.data[4] = 0;
  sendframe.frame.data[5] = 0;
  sendframe.frame.data[6] = 0;
  sendframe.frame.data[7] = 0;

  CommStatus_t ret = send_receive_(sendframe, receiveframe);

  temperature = receiveframe.frame.data[1];
  voltage = uint16_t(receiveframe.frame.data[3] | (((uint16_t)receiveframe.frame.data[4]) << 8));
  errorstate = u_int16_t(receiveframe.frame.data[6] | (((u_int16_t)receiveframe.frame.data[7]) << 8));

  return ret;
}

RMDComm::CommStatus_t RMDComm::readMotorStatus2_(int8_t& temperature, int16_t& iq, int16_t& speed, uint16_t& encoder)
{
  CanDriver::SendFrame_t sendframe;
  CanDriver::ReceiveFrame_t receiveframe;

  sendframe.frame.can_id = 0x140 + 0x1;
  sendframe.frame.can_dlc = 8;
  sendframe.frame.data[0] = 0x9C;
  sendframe.frame.data[1] = 0;
  sendframe.frame.data[2] = 0;
  sendframe.frame.data[3] = 0;
  sendframe.frame.data[4] = 0;
  sendframe.frame.data[5] = 0;
  sendframe.frame.data[6] = 0;
  sendframe.frame.data[7] = 0;

  CommStatus_t ret = send_receive_(sendframe, receiveframe);

  temperature = receiveframe.frame.data[1];

  iq = int16_t(receiveframe.frame.data[2] | (((uint16_t)receiveframe.frame.data[3]) << 8));

  speed = int16_t(receiveframe.frame.data[4] | (((int16_t)receiveframe.frame.data[5]) << 8));

  encoder = uint16_t(receiveframe.frame.data[6] | (((uint16_t)receiveframe.frame.data[7]) << 8));

  return ret;
}

RMDComm::CommStatus_t RMDComm::readMotorStatus3_(int16_t& ia, int16_t& ib, int16_t& ic)
{
  CanDriver::SendFrame_t sendframe;
  CanDriver::ReceiveFrame_t receiveframe;

  sendframe.frame.can_id = 0x140 + 0x1;
  sendframe.frame.can_dlc = 8;
  sendframe.frame.data[0] = 0x9D;
  sendframe.frame.data[1] = 0;
  sendframe.frame.data[2] = 0;
  sendframe.frame.data[3] = 0;
  sendframe.frame.data[4] = 0;
  sendframe.frame.data[5] = 0;
  sendframe.frame.data[6] = 0;
  sendframe.frame.data[7] = 0;

  CommStatus_t ret = send_receive_(sendframe, receiveframe);

  ia = u_int16_t(receiveframe.frame.data[2] | (((u_int16_t)receiveframe.frame.data[3]) << 8));
  ib = u_int16_t(receiveframe.frame.data[4] | (((u_int16_t)receiveframe.frame.data[5]) << 8));
  ic = u_int16_t(receiveframe.frame.data[6] | (((u_int16_t)receiveframe.frame.data[7]) << 8));

  return ret;
}

RMDComm::CommStatus_t RMDComm::speedControlCommand_(const int32_t& speedcontrol, int8_t& temperature, int16_t& iq,
                                                    int16_t& speed, uint16_t& encoder)
{
  CanDriver::SendFrame_t sendframe;
  CanDriver::ReceiveFrame_t receiveframe;

  sendframe.frame.can_id = 0x140 + 0x1;
  sendframe.frame.can_dlc = 8;
  sendframe.frame.data[0] = 0xA2;
  sendframe.frame.data[1] = 0;
  sendframe.frame.data[2] = 0;
  sendframe.frame.data[3] = 0;
  sendframe.frame.data[4] = *((u_int8_t*)(&speedcontrol) + 0);
  sendframe.frame.data[5] = *((u_int8_t*)(&speedcontrol) + 1);
  sendframe.frame.data[6] = *((u_int8_t*)(&speedcontrol) + 2);
  sendframe.frame.data[7] = *((u_int8_t*)(&speedcontrol) + 3);

  CommStatus_t ret = send_receive_(sendframe, receiveframe);

  temperature = receiveframe.frame.data[1];

  iq = u_int16_t(receiveframe.frame.data[2] | (((u_int16_t)receiveframe.frame.data[3]) << 8));

  speed = u_int16_t(receiveframe.frame.data[4] | (((u_int16_t)receiveframe.frame.data[5]) << 8));

  encoder = u_int16_t(receiveframe.frame.data[6] | (((u_int16_t)receiveframe.frame.data[7]) << 8));

  return ret;
}

RMDComm::CommStatus_t RMDComm::positionControlCommand2_(const uint16_t& maxspeed, const int32_t& anglecontrol,
                                                        int8_t& temperature, int16_t& iq, int16_t& speed,
                                                        uint16_t& encoder)
{
  CanDriver::SendFrame_t sendframe;
  CanDriver::ReceiveFrame_t receiveframe;

  sendframe.frame.can_id = 0x140 + 0x1;
  sendframe.frame.can_dlc = 8;
  sendframe.frame.data[0] = 0xA4;
  sendframe.frame.data[1] = 0x00;
  sendframe.frame.data[2] = *((u_int8_t*)(&maxspeed) + 0);
  sendframe.frame.data[3] = *((u_int8_t*)(&maxspeed) + 1);
  sendframe.frame.data[4] = *((u_int8_t*)(&anglecontrol) + 0);
  sendframe.frame.data[5] = *((u_int8_t*)(&anglecontrol) + 1);
  sendframe.frame.data[6] = *((u_int8_t*)(&anglecontrol) + 2);
  sendframe.frame.data[7] = *((u_int8_t*)(&anglecontrol) + 3);

  CommStatus_t ret = send_receive_(sendframe, receiveframe);

  temperature = receiveframe.frame.data[1];

  iq = u_int16_t(receiveframe.frame.data[2] | (((u_int16_t)receiveframe.frame.data[3]) << 8));

  speed = u_int16_t(receiveframe.frame.data[4] | (((u_int16_t)receiveframe.frame.data[5]) << 8));

  encoder = u_int16_t(receiveframe.frame.data[6] | (((u_int16_t)receiveframe.frame.data[7]) << 8));

  return ret;
}

RMDComm::CommStatus_t RMDComm::positionControlCommand4_(const int8_t& spindirection, const uint16_t& maxspeed,
                                                        const int16_t& anglecontrol, int8_t& temperature, int16_t& iq,
                                                        int16_t& speed, uint16_t& encoder)
{
  CanDriver::SendFrame_t sendframe;
  CanDriver::ReceiveFrame_t receiveframe;

  sendframe.frame.can_id = 0x140 + 0x1;
  sendframe.frame.can_dlc = 8;
  sendframe.frame.data[0] = 0xA6;
  sendframe.frame.data[1] = spindirection;
  sendframe.frame.data[2] = *((u_int8_t*)(&maxspeed) + 0);
  sendframe.frame.data[3] = *((u_int8_t*)(&maxspeed) + 1);
  sendframe.frame.data[4] = *((u_int8_t*)(&anglecontrol) + 0);
  sendframe.frame.data[5] = *((u_int8_t*)(&anglecontrol) + 1);
  sendframe.frame.data[6] = 0;
  sendframe.frame.data[7] = 0;

  CommStatus_t ret = send_receive_(sendframe, receiveframe);

  temperature = receiveframe.frame.data[1];

  iq = u_int16_t(receiveframe.frame.data[2] | (((u_int16_t)receiveframe.frame.data[3]) << 8));

  speed = u_int16_t(receiveframe.frame.data[4] | (((u_int16_t)receiveframe.frame.data[5]) << 8));

  encoder = u_int16_t(receiveframe.frame.data[6] | (((u_int16_t)receiveframe.frame.data[7]) << 8));

  return ret;
}

RMDComm::CommStatus_t RMDComm::writeZeroPositionToRom_()
{
  CanDriver::SendFrame_t sendframe;
  CanDriver::ReceiveFrame_t receiveframe;

  sendframe.frame.can_id = 0x140 + 0x1;
  sendframe.frame.can_dlc = 8;
  sendframe.frame.data[0] = 0x19;
  sendframe.frame.data[1] = 0x00;
  sendframe.frame.data[2] = 0x00;
  sendframe.frame.data[3] = 0x00;
  sendframe.frame.data[4] = 0x00;
  sendframe.frame.data[5] = 0x00;
  sendframe.frame.data[6] = 0x00;
  sendframe.frame.data[7] = 0x00;

  CommStatus_t ret = send_receive_(sendframe, receiveframe);

  RCLCPP_DEBUG(node_->get_logger(), "RMDComm: Current position set as Zero position and written in ROM");

  return ret;
}

RMDComm::CommStatus_t RMDComm::motorOffCommand_()
{
  CanDriver::SendFrame_t sendframe;
  CanDriver::ReceiveFrame_t receiveframe;

  sendframe.frame.can_id = 0x140 + 0x1;
  sendframe.frame.can_dlc = 8;
  sendframe.frame.data[0] = 0x80;
  sendframe.frame.data[1] = 0x00;
  sendframe.frame.data[2] = 0x00;
  sendframe.frame.data[3] = 0x00;
  sendframe.frame.data[4] = 0x00;
  sendframe.frame.data[5] = 0x00;
  sendframe.frame.data[6] = 0x00;
  sendframe.frame.data[7] = 0x00;

  CommStatus_t ret = send_receive_(sendframe, receiveframe);

  return ret;
}

RMDComm::CommStatus_t RMDComm::motorStopCommand_()
{
  CanDriver::SendFrame_t sendframe;
  CanDriver::ReceiveFrame_t receiveframe;

  sendframe.frame.can_id = 0x140 + 0x1;
  sendframe.frame.can_dlc = 8;
  sendframe.frame.data[0] = 0x81;
  sendframe.frame.data[1] = 0x00;
  sendframe.frame.data[2] = 0x00;
  sendframe.frame.data[3] = 0x00;
  sendframe.frame.data[4] = 0x00;
  sendframe.frame.data[5] = 0x00;
  sendframe.frame.data[6] = 0x00;
  sendframe.frame.data[7] = 0x00;

  CommStatus_t ret = send_receive_(sendframe, receiveframe);

  return ret;
}

RMDComm::CommStatus_t RMDComm::motorRunningCommand_()
{
  CanDriver::SendFrame_t sendframe;
  CanDriver::ReceiveFrame_t receiveframe;

  sendframe.frame.can_id = 0x140 + 0x1;
  sendframe.frame.can_dlc = 8;
  sendframe.frame.data[0] = 0x88;
  sendframe.frame.data[1] = 0x00;
  sendframe.frame.data[2] = 0x00;
  sendframe.frame.data[3] = 0x00;
  sendframe.frame.data[4] = 0x00;
  sendframe.frame.data[5] = 0x00;
  sendframe.frame.data[6] = 0x00;
  sendframe.frame.data[7] = 0x00;

  CommStatus_t ret = send_receive_(sendframe, receiveframe);

  return ret;
}

RMDComm::CommStatus_t RMDComm::readMultiTurnsAngleCommand_(int64_t& motorangle)
{
  CanDriver::SendFrame_t sendframe;
  CanDriver::ReceiveFrame_t receiveframe;

  sendframe.frame.can_id = 0x140 + 0x1;
  sendframe.frame.can_dlc = 8;
  sendframe.frame.data[0] = 0x92;
  sendframe.frame.data[1] = 0x00;
  sendframe.frame.data[2] = 0x00;
  sendframe.frame.data[3] = 0x00;
  sendframe.frame.data[4] = 0x00;
  sendframe.frame.data[5] = 0x00;
  sendframe.frame.data[6] = 0x00;
  sendframe.frame.data[7] = 0x00;

  CommStatus_t ret = send_receive_(sendframe, receiveframe);

  uint8_t sign_bit = (receiveframe.frame.data[7] & 0x80) >> 7;
  uint8_t sign_mask = 0x0;
  if (sign_bit)
  {
    sign_mask = 0xFF;
  }
  motorangle = int64_t(receiveframe.frame.data[1] | (((int64_t)receiveframe.frame.data[2]) << 8) |
                       (((int64_t)receiveframe.frame.data[3]) << 16) | (((int64_t)receiveframe.frame.data[4]) << 24) |
                       (((int64_t)receiveframe.frame.data[5]) << 32) | (((int64_t)receiveframe.frame.data[6]) << 40) |
                       (((int64_t)receiveframe.frame.data[7]) << 48) | (((int64_t)sign_mask) << 56));
  return ret;
}

RMDComm::CommStatus_t RMDComm::writeEncoderOffsetCommand_(const uint16_t& encoderoffset)
{
  CanDriver::SendFrame_t sendframe;
  CanDriver::ReceiveFrame_t receiveframe;

  sendframe.frame.can_id = 0x140 + 0x1;
  sendframe.frame.can_dlc = 8;
  sendframe.frame.data[0] = 0x91;
  sendframe.frame.data[1] = 0x00;
  sendframe.frame.data[2] = 0x00;
  sendframe.frame.data[3] = 0x00;
  sendframe.frame.data[4] = 0x00;
  sendframe.frame.data[5] = 0x00;
  sendframe.frame.data[6] = *((u_int8_t*)(&encoderoffset) + 0);
  sendframe.frame.data[7] = *((u_int8_t*)(&encoderoffset) + 1);

  CommStatus_t ret = send_receive_(sendframe, receiveframe);

  return ret;
}

RMDComm::CommStatus_t RMDComm::writeAccelData2Ram_(const int32_t& accel)
{
  CanDriver::SendFrame_t sendframe;
  CanDriver::ReceiveFrame_t receiveframe;

  sendframe.frame.can_id = 0x140 + 0x1;
  sendframe.frame.can_dlc = 8;
  sendframe.frame.data[0] = 0x34;
  sendframe.frame.data[1] = 0x00;
  sendframe.frame.data[2] = 0x00;
  sendframe.frame.data[3] = 0x00;
  sendframe.frame.data[4] = *((uint8_t*)(&accel) + 0);
  sendframe.frame.data[5] = *((uint8_t*)(&accel) + 1);
  sendframe.frame.data[6] = *((uint8_t*)(&accel) + 2);
  sendframe.frame.data[7] = *((uint8_t*)(&accel) + 3);

  CommStatus_t ret = send_receive_(sendframe, receiveframe);

  RCLCPP_DEBUG(node_->get_logger(), "RMDComm: Max acceleration set in the actuator RAM.");

  return ret;
}

RMDComm::CommStatus_t RMDComm::torqueCurrentControlCommand_(const int16_t& iqcontrol, int8_t& temperature, int16_t& iq,
                                                            int16_t& speed, uint16_t& encoder)
{
  CanDriver::SendFrame_t sendframe;
  CanDriver::ReceiveFrame_t receiveframe;

  int16_t iq_control_limited = saturate_current_limit_(iqcontrol);
  sendframe.frame.can_id = 0x140 + 0x1;
  sendframe.frame.can_dlc = 8;
  sendframe.frame.data[0] = 0xA1;
  sendframe.frame.data[1] = 0x00;
  sendframe.frame.data[2] = 0x00;
  sendframe.frame.data[3] = 0x00;
  sendframe.frame.data[4] = *((uint8_t*)(&iq_control_limited) + 0);
  sendframe.frame.data[5] = *((uint8_t*)(&iq_control_limited) + 1);
  sendframe.frame.data[6] = 0x00;
  sendframe.frame.data[7] = 0x00;

  CommStatus_t ret = send_receive_(sendframe, receiveframe);

  temperature = receiveframe.frame.data[1];

  iq = u_int16_t(receiveframe.frame.data[2] | (((u_int16_t)receiveframe.frame.data[3]) << 8));

  speed = u_int16_t(receiveframe.frame.data[4] | (((u_int16_t)receiveframe.frame.data[5]) << 8));

  encoder = u_int16_t(receiveframe.frame.data[6] | (((u_int16_t)receiveframe.frame.data[7]) << 8));

  return ret;
}
