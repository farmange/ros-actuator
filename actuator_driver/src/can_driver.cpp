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

CanDriver::CanDriver()
{
}

CanDriver::DriverStatus_t CanDriver::init(const std::string& can_device)
{
  int ret;
  struct ifreq ifr;
  std::string system_str = std::string("sudo ip link set ") + can_device + std::string(" type can bitrate 1000000");
  system(system_str.c_str());
  system_str = std::string("sudo ifconfig ") + can_device + std::string(" up");
  system(system_str.c_str());

  // Create socket
  socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_ < 0)
  {
    perror("socket PF_CAN failed");
    return DRIVER_STATUS_SOCKET_ERR;
  }

  // Specify can device
  strcpy(ifr.ifr_name, can_device.c_str());
  ret = ioctl(socket_, SIOCGIFINDEX, &ifr);
  if (ret < 0)
  {
    perror("ioctl failed");
    return DRIVER_STATUS_IOCTL_ERR;
  }

  // Bind the socket can to send
  addr_.can_family = AF_CAN;
  addr_.can_ifindex = ifr.ifr_ifindex;
  ret = bind(socket_, (struct sockaddr*)&addr_, sizeof(addr_));
  if (ret < 0)
  {
    perror("bind failed");
    return DRIVER_STATUS_BIND_ERR;
  }

  // Disable filtering rules, do not receive packets, only send
  // setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
  int loopback = 0; /* 0 = disabled, 1 = enabled (default) */
  setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

  // Setting for timestamp
  int enabled = 1;
  setsockopt(socket_, SOL_CAN_RAW, SO_TIMESTAMP, &enabled, sizeof(enabled));

  printf("Can initialization done.\n");

  return DRIVER_STATUS_OK;
}

CanDriver::DriverStatus_t CanDriver::sendReceive(const SendFrame_t& sendframe, ReceiveFrame_t& receiveframe)
{
  int nbytes = 1;
  struct timeval tv;
  socklen_t len = sizeof(addr_);

  // Workaround: set the minimum timeout possible to empty AFAP the buffer
  tv.tv_sec = 0;
  tv.tv_usec = 1;
  setTimeOut_(tv);

  // Empty the buffer before reading it
  while (nbytes > 0)
  {
    // printf("Empty the buffer.\n");
    nbytes = recvfrom(socket_, &receiveframe.frame, sizeof(receiveframe.frame), 0, (struct sockaddr*)&addr_, &len);
    // printf("%d bytes read.\n", nbytes);
  }

  // Set back timeout to a value for write or read on socketCan
  tv.tv_usec = 1000;  // 1000
  setTimeOut_(tv);

  // Write on socketCan
  nbytes = write(socket_, &sendframe.frame, sizeof(sendframe.frame));

  if (nbytes != sizeof(sendframe.frame))
  {
    printf("Send Error frame[0]!\r\n");
    return DRIVER_STATUS_WRITE_ERR;
  }

  usleep(10);

  // Read on socketCan
  nbytes = recvfrom(socket_, &receiveframe.frame, sizeof(receiveframe.frame), 0, (struct sockaddr*)&addr_, &len);

  if (nbytes == 0)
  {
    return DRIVER_STATUS_OTHER_ERR;
  }
  else if (nbytes < 1)
  {
    perror("Receive failed");
    return DRIVER_STATUS_READ_ERR;
  }

  return DRIVER_STATUS_OK;
}

CanDriver::DriverStatus_t CanDriver::setTimeOut_(const struct timeval& timeout)
{
  // Settings to get timeout /!\ use after socket creation
  setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
  return DRIVER_STATUS_OK;
}