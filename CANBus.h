/*
***********************************************************************
* CANBus.h: Transmit CAN data using SocketCAN
* This header file can be read by C++ compilers
*
* struct can_frame {
*    canid_t can_id;  // 32 bit CAN_ID + EFF/RTR/ERR flags
*    __u8    can_dlc; // frame payload length in byte (0 .. 8)
*    __u8    __pad;   // padding
*    __u8    __res0;  // reserved
*    __u8    __res1;  // reserved
*    __u8    data[8] __attribute__((aligned(8)));
*  };
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _CANBUS_H_
#define _CANBUS_H_

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <string>

class CANBus {
 public:
  CANBus(const std::string &_port, const std::string &_bitrate)
      : fd_(-1), port_(_port), bitrate_(_bitrate) {
    InitializeSocketCAN();
  }

  int ReceiveOneFrame(struct can_frame *frame) {
    int nbytes = read(fd_, frame, sizeof(struct can_frame));
    if (nbytes < 0) {
      printf("error in SocketCAN read\n");
    }
    return nbytes;
  }  // ReceiveOneFrame

  int SendOneFrame(const struct can_frame *frame) {
    int nbytes = write(fd_, frame, sizeof(struct can_frame));
    if (nbytes < 0) {
      printf("error in SocketCAN write\n");
    }
    return nbytes;
  }  // SendOneFrame

  virtual ~CANBus() { ShutdownSocketCAN(); }

 private:
  int fd_;
  std::string port_;
  std::string bitrate_;

  void InitializeSocketCAN() {
    struct ifreq ifr;

    char sys_buffer[100];
    snprintf(sys_buffer, 100, "sudo ip link set %s up type can bitrate %s",
             port_.c_str(), bitrate_.c_str());
    system(sys_buffer);

    if ((fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
      printf("error in SocketCAN socket\n");
    }

    strcpy(ifr.ifr_name, port_.c_str());
    ioctl(fd_, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      printf("error in SocketCAN bind\n");
    }

    // setup timeout
    /*
    struct timeval tv;
     tv.tv_sec = 0;
     tv.tv_usec = 1000;
     setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);
      */
  }  // InitializeSocketCAN

  void ShutdownSocketCAN() {
    if (close(fd_) < 0) {
      printf("error in SocketCAN close\n");
    }
    system("sudo ip link set can0 down");
  }  // ShutdownSocketCAN

};  // end class CANBus

#endif /* _CANBUS_H_ */