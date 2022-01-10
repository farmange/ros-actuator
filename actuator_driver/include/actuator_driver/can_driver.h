#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
// #include <vector>
#include <string>
// #include <fstream>

class CanDriver
{
public:
  typedef struct receiveFrame_s
  {
    struct can_frame frame;
    struct timeval read_tv;
  } receiveFrame_t;

  typedef struct sendFrame_s
  {
    struct can_frame frame;
  } sendFrame_t;

  CanDriver();

  // Initialise socket and CAN device
  int init(const std::string& can_device);
  int sendReceive(const sendFrame_t& sendframe, receiveFrame_t& receiveframe);

private:
  int setTimeOut_(const struct timeval& timeout);  // /!\ use after socket creation

  int socket_;

  struct timeval tv_;

  struct sockaddr_can addr_;
};

#endif