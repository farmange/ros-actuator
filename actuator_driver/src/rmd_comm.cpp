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

const uint8_t RMDComm::ANGLE_TO_DEG = 100;

RMDComm::RMDComm(rclcpp::Node* node) : BaseComm(node)
{
}

int RMDComm::init(const std::string& can_device, const uint8_t& reduction_ratio, const uint16_t& max_speed,
                  const int32_t& max_accel)
{
  can_interface_.init(can_device);
  // uint32_t max_accel = 700;  // 1dps² / LSB
  writeAccelData2Ram_(max_accel_);
  return 0;
}
/**********/
int RMDComm::sendPosition(const float& joint_position)
{
  int32_t motor_position = degToMotorPos_(joint_position);

  // int16_t maxspeed = 1080;  // 1dps/LSB, 1080/36 = 30 dps (same as partner param);
  int8_t dummy_temperature;
  int16_t dummy_iq;
  int16_t dummy_speed;
  uint16_t dummy_encoder;
  positionControlCommand2_(max_speed_, motor_position, dummy_temperature, dummy_iq, dummy_speed, dummy_encoder);
  return 0;
}
void RMDComm::getCurrentPosition(float position)
{
}
void RMDComm::startHardwareControlLoop()
{
}
void RMDComm::stopHardwareControlLoop()
{
}

void RMDComm::stop()
{
  motorOffCommand_();
}

void RMDComm::start()
{
  motorRunningCommand_();
}

int RMDComm::getState(float& temperature, float& iq, float& ia, float& ib, float& ic, float& position, float& speed,
                      float& voltage, std::string& error)
{
  int8_t raw_temperature;
  uint8_t raw_errorstate;
  int16_t raw_iq, raw_speed;
  int16_t raw_ia, raw_ib, raw_ic;
  uint16_t raw_voltage;
  uint16_t raw_encoder;
  int32_t raw_current_position = 0;

  // Read motor multi turn angle
  int64_t dummy_angle = 0;
  readMultiTurnsAngleCommand_(dummy_angle);
  raw_current_position = static_cast<int32_t>(dummy_angle);

  // Read motor error status, voltage and temperature
  readMotorStatus1_(raw_temperature, raw_voltage, raw_errorstate);

  // Read motor temperature, iq, speed and encoder
  readMotorStatus2_(raw_temperature, raw_iq, raw_speed, raw_encoder);

  // Read 3 phases current
  readMotorStatus3_(raw_ia, raw_ib, raw_ic);

  temperature = raw_temperature;
  iq = ((raw_iq * 33.) / 2048.);
  ia = (raw_ia / 64.);
  ib = (raw_ib / 64.);
  ic = (raw_ic / 64.);
  position = motorPosToDeg_(raw_current_position);
  speed = (raw_speed / 36.);
  voltage = (raw_voltage / 10.);
  error = std::to_string(raw_errorstate);

  if (raw_errorstate != 0)
  {
    return 1;
  }
  return 0;
}

// Return the motor position in deg from the joint position
int32_t RMDComm::degToMotorPos_(const double& joint_pos)
{
  return (int32_t)(joint_pos * reduction_ratio_ * ANGLE_TO_DEG);
};

// Return the joint position in deg from the motor position
double RMDComm::motorPosToDeg_(const int32_t& motor_pos)
{
  return (((double)motor_pos) / (reduction_ratio_ * ANGLE_TO_DEG));
};

int RMDComm::readMotorStatus1_(int8_t& temperature, uint16_t& voltage, uint8_t& errorstate)
{
  CanDriver::sendFrame_t sendframe;
  CanDriver::receiveFrame_t receiveframe;

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

  can_interface_.sendReceive(sendframe, receiveframe);

  temperature = receiveframe.frame.data[1];
  voltage = uint16_t(receiveframe.frame.data[3] | (((uint16_t)receiveframe.frame.data[4]) << 8));
  errorstate = receiveframe.frame.data[7];

  return 0;
}

int RMDComm::readMotorStatus2_(int8_t& temperature, int16_t& iq, int16_t& speed, uint16_t& encoder)
{
  CanDriver::sendFrame_t sendframe;
  CanDriver::receiveFrame_t receiveframe;

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

  can_interface_.sendReceive(sendframe, receiveframe);

  temperature = receiveframe.frame.data[1];

  iq = int16_t(receiveframe.frame.data[2] | (((uint16_t)receiveframe.frame.data[3]) << 8));

  speed = int16_t(receiveframe.frame.data[4] | (((int16_t)receiveframe.frame.data[5]) << 8));

  encoder = uint16_t(receiveframe.frame.data[6] | (((uint16_t)receiveframe.frame.data[7]) << 8));

  return 0;
}

int RMDComm::readMotorStatus3_(int16_t& ia, int16_t& ib, int16_t& ic)
{
  CanDriver::sendFrame_t sendframe;
  CanDriver::receiveFrame_t receiveframe;

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

  can_interface_.sendReceive(sendframe, receiveframe);

  ia = u_int16_t(receiveframe.frame.data[2] | (((u_int16_t)receiveframe.frame.data[3]) << 8));
  ib = u_int16_t(receiveframe.frame.data[4] | (((u_int16_t)receiveframe.frame.data[5]) << 8));
  ic = u_int16_t(receiveframe.frame.data[6] | (((u_int16_t)receiveframe.frame.data[7]) << 8));

  return 0;
}

int RMDComm::speedControlCommand_(const int32_t& speedcontrol, int8_t& temperature, int16_t& iq, int16_t& speed,
                                  int16_t& encoder)
{
  CanDriver::sendFrame_t sendframe;
  CanDriver::receiveFrame_t receiveframe;

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

  can_interface_.sendReceive(sendframe, receiveframe);

  temperature = receiveframe.frame.data[1];

  iq = u_int16_t(receiveframe.frame.data[2] | (((u_int16_t)receiveframe.frame.data[3]) << 8));

  speed = u_int16_t(receiveframe.frame.data[4] | (((u_int16_t)receiveframe.frame.data[5]) << 8));

  encoder = u_int16_t(receiveframe.frame.data[6] | (((u_int16_t)receiveframe.frame.data[7]) << 8));

  return 0;
}

int RMDComm::positionControlCommand2_(const uint16_t& maxspeed, const int32_t& anglecontrol, int8_t& temperature,
                                      int16_t& iq, int16_t& speed, uint16_t& encoder)
{
  CanDriver::sendFrame_t sendframe;
  CanDriver::receiveFrame_t receiveframe;

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

  can_interface_.sendReceive(sendframe, receiveframe);

  temperature = receiveframe.frame.data[1];

  iq = u_int16_t(receiveframe.frame.data[2] | (((u_int16_t)receiveframe.frame.data[3]) << 8));

  speed = u_int16_t(receiveframe.frame.data[4] | (((u_int16_t)receiveframe.frame.data[5]) << 8));

  encoder = u_int16_t(receiveframe.frame.data[6] | (((u_int16_t)receiveframe.frame.data[7]) << 8));

  return 0;
}

int RMDComm::positionControlCommand4_(const int8_t& spindirection, const uint16_t& maxspeed,
                                      const int16_t& anglecontrol, int8_t& temperature, int16_t& iq, int16_t& speed,
                                      uint16_t& encoder)
{
  CanDriver::sendFrame_t sendframe;
  CanDriver::receiveFrame_t receiveframe;

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

  can_interface_.sendReceive(sendframe, receiveframe);

  encoder = u_int16_t(receiveframe.frame.data[6] | (((u_int16_t)receiveframe.frame.data[7]) << 8));

  return 0;
}

int RMDComm::writeZeroPositionToRom_()
{
  CanDriver::sendFrame_t sendframe;
  CanDriver::receiveFrame_t receiveframe;

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

  can_interface_.sendReceive(sendframe, receiveframe);
  printf("Current position set as Zero position and written in ROM\n");

  return 0;
}

int RMDComm::motorOffCommand_()
{
  CanDriver::sendFrame_t sendframe;
  CanDriver::receiveFrame_t receiveframe;

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

  can_interface_.sendReceive(sendframe, receiveframe);

  return 0;
}

int RMDComm::motorStopCommand_()
{
  CanDriver::sendFrame_t sendframe;
  CanDriver::receiveFrame_t receiveframe;

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

  can_interface_.sendReceive(sendframe, receiveframe);

  return 0;
}

int RMDComm::motorRunningCommand_()
{
  CanDriver::sendFrame_t sendframe;
  CanDriver::receiveFrame_t receiveframe;

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

  can_interface_.sendReceive(sendframe, receiveframe);

  return 0;
}

int RMDComm::readMultiTurnsAngleCommand_(int64_t& motorangle)
{
  CanDriver::sendFrame_t sendframe;
  CanDriver::receiveFrame_t receiveframe;

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

  can_interface_.sendReceive(sendframe, receiveframe);

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
  return 0;
}

int RMDComm::writeEncoderOffsetCommand_(const uint16_t& encoderoffset)
{
  CanDriver::sendFrame_t sendframe;
  CanDriver::receiveFrame_t receiveframe;

  sendframe.frame.can_id = 0x140 + 0x1;
  sendframe.frame.can_dlc = 8;
  sendframe.frame.data[0] = 0x91;
  sendframe.frame.data[1] = 0x00;
  sendframe.frame.data[2] = 0x00;
  sendframe.frame.data[3] = 0x00;
  sendframe.frame.data[4] = 0x00;
  sendframe.frame.data[5] = 0x00;
  sendframe.frame.data[6] = 0x00;
  sendframe.frame.data[7] = 0x00;

  can_interface_.sendReceive(sendframe, receiveframe);

  return 0;
}

int RMDComm::writeAccelData2Ram_(const int32_t& accel)
{
  CanDriver::sendFrame_t sendframe;
  CanDriver::receiveFrame_t receiveframe;

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

  can_interface_.sendReceive(sendframe, receiveframe);
  printf("Max acceleration set in the actuator RAM.\n");
  return 0;
}

int RMDComm::torqueCurrentControlCommand_(const int16_t& iqcontrol, int16_t& iq)
{
  CanDriver::sendFrame_t sendframe;
  CanDriver::receiveFrame_t receiveframe;

  sendframe.frame.can_id = 0x140 + 0x1;
  sendframe.frame.can_dlc = 8;
  sendframe.frame.data[0] = 0xA1;
  sendframe.frame.data[1] = 0x00;
  sendframe.frame.data[2] = 0x00;
  sendframe.frame.data[3] = 0x00;
  sendframe.frame.data[4] = *((uint8_t*)(&iqcontrol) + 0);
  sendframe.frame.data[5] = *((uint8_t*)(&iqcontrol) + 1);
  sendframe.frame.data[6] = 0x00;
  sendframe.frame.data[7] = 0x00;

  can_interface_.sendReceive(sendframe, receiveframe);

  iq = uint16_t(receiveframe.frame.data[2] | (((uint16_t)receiveframe.frame.data[3]) << 8));

  return 0;
}
