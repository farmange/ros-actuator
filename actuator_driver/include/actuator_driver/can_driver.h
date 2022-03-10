#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <string>

#include "rclcpp/rclcpp.hpp"

class CanDriver
{
public:
  typedef struct ReceiveFrame_s
  {
    struct can_frame frame;
    struct timeval read_tv;
  } ReceiveFrame_t;

  typedef struct SendFrame_s
  {
    struct can_frame frame;
  } SendFrame_t;

  typedef enum DriverStatus_e : uint8_t
  {
    DRIVER_STATUS_OK = 0,
    DRIVER_STATUS_SOCKET_ERR,
    DRIVER_STATUS_IOCTL_ERR,
    DRIVER_STATUS_BIND_ERR,
    DRIVER_STATUS_READ_ERR,
    DRIVER_STATUS_WRITE_ERR,
    DRIVER_STATUS_OTHER_ERR
  } DriverStatus_t;

  CanDriver();
  ~CanDriver();
  // Initialise socket and CAN device
  DriverStatus_t init(const std::string& can_device);
  DriverStatus_t sendReceive(const SendFrame_t& sendframe, ReceiveFrame_t& receiveframe);

private:
  DriverStatus_t setTimeOut_(const struct timeval& timeout);  // /!\ use after socket creation
  std::string can_device_;
  int socket_;
  struct timeval tv_;
  struct sockaddr_can addr_;
  socklen_t addr_len_;
};

#endif