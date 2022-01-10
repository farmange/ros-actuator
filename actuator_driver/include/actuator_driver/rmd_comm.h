#ifndef RMD_COMM_H
#define RMD_COMM_H

#include <stdint.h>

#include "rclcpp/rclcpp.hpp"

#include "actuator_driver/can_driver.h"
#include "actuator_driver/base_comm.h"

class RMDComm : public BaseComm
{
  static const uint8_t ANGLE_TO_DEG;

public:
  RMDComm(rclcpp::Node* node);

  /* TODO : interface impl */
  int init(const std::string& can_device, const uint8_t& reduction_ratio, const uint16_t& max_speed,
           const int32_t& max_accel);
  // Initialise socket and CAN device
  int sendPosition(const float& cmd);
  void getCurrentPosition(float position);
  void startHardwareControlLoop();
  void stopHardwareControlLoop();
  void stop();
  void start();

  int getState(float& temperature, float& iq, float& ia, float& ib, float& ic, float& position, float& speed,
               float& voltage, std::string& error);

private:
  int32_t degToMotorPos_(const double& joint_pos);

  // Return the joint position in deg from the motor position
  double motorPosToDeg_(const int32_t& motor_pos);

  // Read the motor's error status and voltage, temperature and
  // other information
  int readMotorStatus1_(int8_t& temperature1, uint16_t& voltage, uint8_t& errorstate);

  // Read motor temperature, voltage, speed, encoder position
  int readMotorStatus2_(int8_t& temperature2, int16_t& iq, int16_t& speed, uint16_t& encoder);

  // Read the phase current status data of the motor
  int readMotorStatus3_(int16_t& ia, int16_t& ib, int16_t& ic);

  // Turn off motor, while clearing the motor operating status and
  // previously received control commands
  int motorOffCommand_();

  // Stop motor, but do not clear the motor operating state and
  // previously received control commands
  int motorStopCommand_();

  // Resume motor operation from motor stop command (Recovery
  // control mode before stop motor )
  int motorRunningCommand_();

  // The host sends this command to control the position of the motor
  //(multi-turn angle) and set a maximum speed.
  int positionControlCommand2_(const uint16_t& maxspeed, const int32_t& anglecontrol, int8_t& temperature, int16_t& iq,
                               int16_t& speed, uint16_t& encoder);

  // The host sends this command to control the position of the
  // motor (single-turn angle) w/ speed & spin direction
  int positionControlCommand4_(const int8_t& spindirection, const uint16_t& maxspeed, const int16_t& anglecontrol,
                               int8_t& temperature, int16_t& iq, int16_t& speed, uint16_t& encoder);

  int speedControlCommand_(const int32_t& speedcontrol, int8_t& temperature, int16_t& iq, int16_t& speed,
                           int16_t& encoder);
  // The host sends this command to control the speed of the motor.
  // Speed Control is int32_t, which corresponds to the actual speed of 0.01dps/LSB.

  int writeZeroPositionToRom_();
  // This command needs to be powered on again to take effect.
  // It will write the 0 position to ROM

  // The host sends command to read the multi-turn angle of the motor
  int readMultiTurnsAngleCommand_(int64_t& motorangle);

  // The host sends the command to set encoder offset
  int writeEncoderOffsetCommand_(const uint16_t& encoderoffset);

  // The host sends the command to write the acceleration to the RAM, and the
  // write parameters are invalid after the power is turned off.
  int writeAccelData2Ram_(const int32_t& accel);

  // The host sends the command to control torque current output of the motor. Iq Control is int16_t type, the value
  // range: -2000~2000, corresponding to the actual torque current range -32A~32A
  int torqueCurrentControlCommand_(const int16_t& iqcontrol, int16_t& iq);

private:
  CanDriver can_interface_;
};

#endif