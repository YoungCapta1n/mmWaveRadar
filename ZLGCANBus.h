/*
***********************************************************************
* ZLGCANBus.h: Transmit CAN data using ZLG
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

#ifndef _ZLGCANBUS_H_
#define _ZLGCANBUS_H_

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

#include "controlcan.h"

class ZLGCANBus {
 public:
  ZLGCANBus(const int _bitrate) : bitrate_(_bitrate) { InitializeSocketCAN(); }

  int ReceiveOneFrame(struct can_frame *frame) {
    int reclen = 0;
    VCI_CAN_OBJ rec[3000];  //接收缓存，设为3000为佳。

    if ((reclen = VCI_Receive(VCI_USBCAN2, 0, 0, rec, 3000, 0)) > 0) {
      frame->can_id = rec[0].ID;
      frame->can_dlc = rec[0].DataLen;
      memcpy(frame->data, rec[0].Data, 8);
    }

    return reclen;

  }  // ReceiveOneFrame

  int SendOneFrame(const struct can_frame *frame) {
    VCI_CAN_OBJ vco[1];
    vco[0].ID = frame->can_id;
    vco[0].RemoteFlag = 0;
    vco[0].ExternFlag = 0;
    vco[0].DataLen = frame->can_dlc;
    memcpy(vco[0].Data, frame->data, 8);

    int nbytes = VCI_Transmit(VCI_USBCAN2, 0, 0, vco, 1);
    if (nbytes < 0) {
      printf("error in SocketCAN write\n");
    }
    return nbytes;
  }  // SendOneFrame

  virtual ~ZLGCANBus() { ShutdownSocketCAN(); }

 private:
  int bitrate_;

  int InitializeSocketCAN() {
    VCI_BOARD_INFO pInfo1[50];

    VCI_FindUsbDevice2(pInfo1);

    if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) == 1)  //打开设备
    {
      printf(">>open deivce success!\n");  //打开设备成功
    } else {
      printf(">>open deivce error!\n");
      return 1;
    }

    //初始化参数，严格参数二次开发函数库说明书。
    VCI_INIT_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 1;  //接收所有帧

    switch (bitrate_) {
      case 10000:
        config.Timing0 = 0x31;
        config.Timing1 = 0x1C;
        break;
      case 20000:
        config.Timing0 = 0x18;
        config.Timing1 = 0x1C;
        break;
      case 40000:
        config.Timing0 = 0x87;
        config.Timing1 = 0xFF;
        break;
      case 50000:
        config.Timing0 = 0x09;
        config.Timing1 = 0x1C;
        break;
      case 80000:
        config.Timing0 = 0x83;
        config.Timing1 = 0xFF;
        break;
      case 100000:
        config.Timing0 = 0x04;
        config.Timing1 = 0x1C;
        break;
      case 125000:
        config.Timing0 = 0x03;
        config.Timing1 = 0x1C;
        break;
      case 200000:
        config.Timing0 = 0x81;
        config.Timing1 = 0xFA;
        break;
      case 250000:
        config.Timing0 = 0x01;
        config.Timing1 = 0x1C;
        break;
      case 400000:
        config.Timing0 = 0x80;
        config.Timing1 = 0xFA;
        break;
      case 500000:
        config.Timing0 = 0x00;
        config.Timing1 = 0x1C;
        break;
      case 800000:
        config.Timing0 = 0x00;
        config.Timing1 = 0x16;
        break;
      case 1000000:
        config.Timing0 = 0x00;
        config.Timing1 = 0x14;
        break;
      default:
        config.Timing0 = 0x00;
        config.Timing1 = 0x1C;
        break;
    }

    config.Mode = 0;  //正常模式

    if (VCI_InitCAN(VCI_USBCAN2, 0, 0, &config) != 1) {
      printf(">>Init CAN1 error\n");
      VCI_CloseDevice(VCI_USBCAN2, 0);
    }

    if (VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1) {
      printf(">>Start CAN1 error\n");
      VCI_CloseDevice(VCI_USBCAN2, 0);
    }

    return 0;
  }  // InitializeSocketCAN

  void ShutdownSocketCAN() {
    VCI_ResetCAN(VCI_USBCAN2, 0, 0);  //复位CAN1通道。
    usleep(100000);                   //延时100ms。
    VCI_ResetCAN(VCI_USBCAN2, 0, 1);  //复位CAN2通道。
    usleep(100000);                   //延时100ms。
    VCI_CloseDevice(VCI_USBCAN2, 0);  //关闭设备。
  }                                   // ShutdownSocketCAN

};  // end class CANBus

#endif /* _ZLGCANBUS_H_ */