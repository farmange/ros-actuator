#ifndef RMD_COMM_H
#define RMD_COMM_H

#include <stdint.h>

#include "rclcpp/rclcpp.hpp"

#include "actuator_driver/can_driver.h"
#include "actuator_driver/base_comm.h"

class RMDComm : public BaseComm
{
  static const uint8_t ANGLE_TO_0_01DEG;
  static const uint8_t SPEED_TO_0_01DPS;
  static const float TORQUE_CONSTANT;

public:
  RMDComm(rclcpp_lifecycle::LifecycleNode* node);

  /* TODO : interface impl */
  CommStatus_t init(const std::string& can_device, const uint8_t& reduction_ratio, const uint16_t& max_speed,
                    const int32_t& max_accel, const float& current_limit);
  // Initialise socket and CAN device
  CommStatus_t sendPosition(const float& joint_position_cmd);
  CommStatus_t sendTorque(const float& joint_torque_cmd);
  CommStatus_t sendSpeed(const float& joint_speed_cmd);

  CommStatus_t stop();
  CommStatus_t start();

  CommStatus_t getState(float& temperature, float& torque, float& ia, float& ib, float& ic, float& iq, float& position,
                        float& speed, float& voltage, uint16_t& error);

private:
  int32_t conv_joint_deg_to_motor_pos_(const double& joint_deg) const;
  double conv_motor_pos_to_joint_deg_(const int32_t& motor_pos) const;

  int32_t conv_joint_speed_to_motor_speed_(const double& joint_deg) const;

  int16_t conv_joint_torque_to_iq_(const float& torque) const;
  float conv_iq_to_joint_torque_(const int16_t& iq) const;

  float conv_iq_to_ampere_(const int16_t& iq) const;
  int16_t conv_ampere_to_iq_(const float& ampere) const;

  float conv_iabc_to_ampere_(const int16_t& iabc) const;
  float conv_motor_speed_to_joint_dps_(const int16_t& motor_speed) const;
  float conv_voltage_to_volt_(const uint16_t& voltage) const;

  int16_t saturate_current_limit_(const int16_t& iq) const;

  CommStatus_t send_receive_(const CanDriver::SendFrame_t& send_frame, CanDriver::ReceiveFrame_t& receive_frame);

  // Read the motor's error status and voltage, temperature and
  // other information
  CommStatus_t readMotorStatus1_(int8_t& temperature1, uint16_t& voltage, uint16_t& errorstate);

  // Read motor temperature, voltage, speed, encoder position
  CommStatus_t readMotorStatus2_(int8_t& temperature2, int16_t& iq, int16_t& speed, uint16_t& encoder);

  // Read the phase current status data of the motor
  CommStatus_t readMotorStatus3_(int16_t& ia, int16_t& ib, int16_t& ic);

  // Turn off motor, while clearing the motor operating status and
  // previously received control commands
  CommStatus_t motorOffCommand_();

  // Stop motor, but do not clear the motor operating state and
  // previously received control commands
  CommStatus_t motorStopCommand_();

  // Resume motor operation from motor stop command (Recovery
  // control mode before stop motor )
  CommStatus_t motorRunningCommand_();

  // The host sends this command to control the position of the motor
  //(multi-turn angle) and set a maximum speed.
  CommStatus_t positionControlCommand2_(const uint16_t& maxspeed, const int32_t& anglecontrol, int8_t& temperature,
                                        int16_t& iq, int16_t& speed, uint16_t& encoder);

  // The host sends this command to control the position of the
  // motor (single-turn angle) w/ speed & spin direction
  CommStatus_t positionControlCommand4_(const int8_t& spindirection, const uint16_t& maxspeed,
                                        const int16_t& anglecontrol, int8_t& temperature, int16_t& iq, int16_t& speed,
                                        uint16_t& encoder);

  CommStatus_t speedControlCommand_(const int32_t& speedcontrol, int8_t& temperature, int16_t& iq, int16_t& speed,
                                    uint16_t& encoder);
  // The host sends this command to control the speed of the motor.
  // Speed Control is int32_t, which corresponds to the actual speed of 0.01dps/LSB.

  CommStatus_t writeZeroPositionToRom_();
  // This command needs to be powered on again to take effect.
  // It will write the 0 position to ROM

  // The host sends command to read the multi-turn angle of the motor
  CommStatus_t readMultiTurnsAngleCommand_(int64_t& motorangle);

  // The host sends the command to set encoder offset
  CommStatus_t writeEncoderOffsetCommand_(const uint16_t& encoderoffset);

  // The host sends the command to write the acceleration to the RAM, and the
  // write parameters are invalid after the power is turned off.
  CommStatus_t writeAccelData2Ram_(const int32_t& accel);

  // The host sends the command to control torque current output of the motor. Iq Control is int16_t type, the value
  // range: -2000~2000, corresponding to the actual torque current range -32A~32A
  CommStatus_t torqueCurrentControlCommand_(const int16_t& iqcontrol, int8_t& temperature, int16_t& iq, int16_t& speed,
                                            uint16_t& encoder);

private:
  CanDriver can_interface_;
};

#endif