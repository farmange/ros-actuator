#include <cstdio>
#include <stdlib.h>
// #include <iostream>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/sockios.h>
// #include <sys/types.h>

#include "actuator_driver/can_driver.h"
// #include "utils.h"

#include <iostream>

CanDriver::CanDriver()
{
}

CanDriver::~CanDriver()
{
  // std::string system_str = std::string("sudo ip link set ") + can_device_ + std::string(" down");
  // system(system_str.c_str());
}

CanDriver::DriverStatus_t CanDriver::init(const std::string& can_device)
{
  int ret;
  struct ifreq ifr;
  can_device_ = can_device;
  std::string system_str;
  system_str = std::string("sudo ip link set ") + can_device_ + std::string(" down");
  system(system_str.c_str());
  system_str = std::string("sudo ip link set ") + can_device_ + std::string(" type can bitrate 1000000");
  system(system_str.c_str());
  system_str = std::string("sudo ip link set ") + can_device_ + std::string(" up");
  system(system_str.c_str());

  // Create socket
  socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("CanDriver"), "socket PF_CAN failed !");
    return DRIVER_STATUS_SOCKET_ERR;
  }

  // Specify can device
  strcpy(ifr.ifr_name, can_device_.c_str());
  ret = ioctl(socket_, SIOCGIFINDEX, &ifr);
  if (ret < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("CanDriver"), "IOCTL SIOCGIFINDEX failed !");
    return DRIVER_STATUS_IOCTL_ERR;
  }

  // Bind the socket can to send
  addr_.can_family = AF_CAN;
  addr_.can_ifindex = ifr.ifr_ifindex;
  ret = bind(socket_, (struct sockaddr*)&addr_, sizeof(addr_));
  if (ret < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("CanDriver"), "Bind failed !");
    return DRIVER_STATUS_BIND_ERR;
  }

  // Disable filtering rules, do not receive packets, only send
  // setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
  int loopback = 0; /* 0 = disabled, 1 = enabled (default) */
  setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

  // Setting for timestamp
  int enabled = 1;
  setsockopt(socket_, SOL_CAN_RAW, SO_TIMESTAMP, &enabled, sizeof(enabled));

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 2;
  setTimeOut_(tv);

  addr_len_ = sizeof(addr_);

  RCLCPP_DEBUG(rclcpp::get_logger("CanDriver"), "Can initialization done.");

  return DRIVER_STATUS_OK;
}

CanDriver::DriverStatus_t CanDriver::sendReceive(const SendFrame_t& sendframe, ReceiveFrame_t& receiveframe)
{
  int nbytes = 1;

  // Write on socketCan
  nbytes = write(socket_, &sendframe.frame, sizeof(sendframe.frame));

  if (nbytes != sizeof(sendframe.frame))
  {
    RCLCPP_ERROR(rclcpp::get_logger("CanDriver"), "Send Error frame : nbytes != sizeof(sendframe.frame) !");
    return DRIVER_STATUS_WRITE_ERR;
  }

  // Read on socketCan
  nbytes = recvfrom(socket_, &receiveframe.frame, sizeof(receiveframe.frame), 0, (struct sockaddr*)&addr_, &addr_len_);

  if (nbytes != sizeof(sendframe.frame))
  {
    if (nbytes < 1)
    {
      RCLCPP_ERROR(rclcpp::get_logger("CanDriver"), "Receive failed !");
      return DRIVER_STATUS_READ_ERR;
    }
    else
    {
      return DRIVER_STATUS_OTHER_ERR;
    }
  }

  if (receiveframe.frame.data[0] != sendframe.frame.data[0])
  {
    return DRIVER_STATUS_OTHER_ERR;
  }

  return DRIVER_STATUS_OK;
}

CanDriver::DriverStatus_t CanDriver::setTimeOut_(const struct timeval& timeout)
{
  // Settings to get timeout /!\ use after socket creation
  setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
  return DRIVER_STATUS_OK;
}