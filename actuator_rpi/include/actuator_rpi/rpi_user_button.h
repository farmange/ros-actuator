//============================================================================
// Name        : armms_user_button.h
// Author      : Florian Armange, ORTHOPUS
// Version     : 0.0
// Copyright   : LGPLv3
// Description : Class definition that bind all low level function
//               of the libkinovadrv
//============================================================================

#ifndef ARMMS_RPI_USER_BUTTON_H
#define ARMMS_RPI_USER_BUTTON_H

#include <ros/ros.h>
#include <wiringPi.h>

namespace armms_rpi
{
class ArmmsUserButton
{
public:
  ArmmsUserButton(const ros::NodeHandle& nh);
  void update(bool& button_up, bool& button_down);

private:
  ros::NodeHandle nh_;

  int btn_up_pin_;
  int btn_down_pin_;

  void retrieveParameters_();
};

}  // namespace armms_rpi
#endif
