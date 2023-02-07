/*
****************************************************************************
* ContiRadarData.h:
* Data of Continental mmWaveRadar (ARS40X), etc.
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#ifndef _CONTIRADARDATA_H_
#define _CONTIRADARDATA_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

struct ContiRadarConfig {
  uint8_t SensorID;
  uint16_t MaxDistance;
  uint8_t RadarPower;
  uint8_t OutputType;
  uint8_t SendQuality;
  uint8_t SendExtInfo;
  uint8_t SortIndex;
  uint8_t CtrlRelay;
  uint8_t RCS_Threshold;
};  // ContiRadarConfig

struct ContiRadarRTConfig {
  bool NVMReadStatus;
  uint8_t SensorID;
  uint8_t RadarError;
  bool SendQualityCfg;
  bool SendExtInfoCfg;
  uint8_t MotionRxState;
};  // ContiRadarRTConfig

template <std::size_t max_num_object>
struct RadarObjectInfoARS {
  std::size_t num_of_object;
  std::array<double, max_num_object> Obj_timestamp;  // unit: ms
  std::array<bool, max_num_object> Obj_IsValid;      //
  std::array<uint8_t, max_num_object> Obj_ID;
  std::array<float, max_num_object> Obj_DistLong;
  std::array<float, max_num_object> Obj_DistLat;
  std::array<float, max_num_object> Obj_VrelLong;
  std::array<float, max_num_object> Obj_VrelLat;
  std::array<uint8_t, max_num_object> Obj_DynProp;
  std::array<float, max_num_object> Obj_RCS;
  std::array<uint8_t, max_num_object> Obj_DistLong_rms;
  std::array<uint8_t, max_num_object> Obj_DistLat_rms;
  std::array<uint8_t, max_num_object> Obj_VrelLong_rms;
  std::array<uint8_t, max_num_object> Obj_VrelLat_rms;
  std::array<uint8_t, max_num_object> Obj_ArelLong_rms;
  std::array<uint8_t, max_num_object> Obj_ArelLat_rms;
  std::array<uint8_t, max_num_object> Obj_Orientation_rms;
  std::array<uint8_t, max_num_object> Obj_MeasState;
  std::array<uint8_t, max_num_object> Obj_ProbOfExist;
  std::array<float, max_num_object> Obj_ArelLong;
  std::array<float, max_num_object> Obj_ArelLat;
  std::array<uint8_t, max_num_object> Obj_Class;
  std::array<float, max_num_object> Obj_OrientationAngel;
  std::array<float, max_num_object> Obj_Length;
  std::array<float, max_num_object> Obj_Width;
  std::array<uint8_t, max_num_object> Obj_CollDetRegionBitfield;
};  // RadarObjectInfoARS

struct RadarClusterInfoARS {
  double Cluster_timestamp;  // unit: ms
  uint32_t Cluster_ID;
  float Cluster_DistLong;
  float Cluster_DistLat;
  float Cluster_VrelLong;
  float Cluster_VrelLat;
  uint32_t Cluster_DynProp;
  float Cluster_RCS;
  uint32_t Cluster_DistLong_rms;
  uint32_t Cluster_DistLat_rms;
  uint32_t Cluster_VrelLong_rms;
  uint32_t Cluster_VrelLat_rms;
  uint32_t Cluster_Pdh0;
  uint32_t Cluster_AmbigState;
  uint32_t Cluster_InvalidState;
};  // RadarClusterInfoARS

#endif /* _CONTIRADARDATA_H_ */
