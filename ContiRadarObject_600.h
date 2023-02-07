/*
****************************************************************************
* ContiRadarObject_600.h:
* Receive Data from Continental mmWaveRadar (ARS40X), etc.
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#ifndef _CONTIRADAROBJECT_600_H_
#define _CONTIRADAROBJECT_600_H_

#include <linux/can.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>

#include "ContiRadarData.h"

template <std::size_t max_num_object>
class ContiRadarObject_600 {
  struct RadarSpeedInputARS {
    uint8_t Radar_SpeedDirection;
    uint16_t Radar_SpeedValue;
  };  // RadarSpeedInputARS
  struct RadarYawRateInputARS {
    uint16_t Radar_YawRateValue;
  };  // RadarYawRateInputARS

 public:
  ContiRadarObject_600(const uint8_t sensorID,
                       const double wait_time_object_status = 40)
      : wait_time_object_status_(wait_time_object_status),
        RadarState_ID_(0x201),
        SpeedInformation_ID_(0x300),
        YawRateInformation_ID_(0x301),
        Obj_0_Status_ID_(0x60A),
        Obj_1_General_ID_(0x60B),
        Obj_2_Quality_ID_(0x60C),
        Obj_3_Extended_ID_(0x60D),
        Obj_4_Warning_ID_(0x60E),
        radar_object_list_({
            0,        // num_of_object
            {0},      // Obj_timestamp
            {false},  // Obj_IsValid
            {0},      // Obj_ID
            {0},      // Obj_DistLong
            {0},      // Obj_DistLat
            {0},      // Obj_VrelLong
            {0},      // Obj_VrelLat
            {0},      // Obj_DynProp
            {0},      // Obj_RCS
            {0},      // Obj_DistLong_rms
            {0},      // Obj_DistLat_rms
            {0},      // Obj_VrelLong_rms
            {0},      // Obj_VrelLat_rms
            {0},      // Obj_ArelLong_rms
            {0},      // Obj_ArelLat_rms
            {0},      // Obj_Orientation_rms
            {0},      // Obj_MeasState
            {0},      // Obj_ProbOfExist
            {0},      // Obj_ArelLong
            {0},      // Obj_ArelLat
            {0},      // Obj_Class
            {0},      // Obj_OrientationAngel
            {0},      // Obj_Length
            {0},      // Obj_Width
            {0}       // Obj_CollDetRegionBitfield
        }),
        radar_object_index_(0),
        obj_status_recv_time_(0.0),
        valid_obj_ID_({}),
        radar_config_status_({
            false,  // NVMReadStatus
            0x0,    // SensorID
            0x0,    // RadarError
            false,  // SendQualityCfg
            false,  // SendExtInfoCfg
            0x0     // MotionRxState
        }),
        RadarSpeed_Input_({
            0,  // Radar_SpeedDirection
            0   // Radar_SpeedValue
        }),
        RadarYawRate_Input_({
            0  // Radar_YawRateValue
        }) {
    InitMessageID(sensorID);
  }
  virtual ~ContiRadarObject_600() = default;

  bool Check_Is_Canbus_Idle_RegenerateObject() {
    if (IsObjectFrameRecv.load(std::memory_order_relaxed)) {
      // when object frame is received (canbus is busy)
      IsObjectFrameRecv.store(false, std::memory_order_relaxed);
      return false;
    }

    double delta_ts = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - driver_pt_start_)
                          .count() -
                      obj_status_recv_time_;

    if (delta_ts >= wait_time_object_status_) {
      radar_object_list_.num_of_object = radar_object_index_ > max_num_object
                                             ? max_num_object
                                             : radar_object_index_;
      return true;
    }

    return false;
  }  // Check_Is_Canbus_Idle_RegenerateObject

  // parse radar data from can frame
  ContiRadarObject_600 &ParseRadarData(const struct can_frame *candata) {
    uint16_t id_normal = static_cast<uint16_t>(candata->can_id);

    if (id_normal == RadarState_ID_) {
      radar_config_status_ = HandleConfigStatus(candata->data);
    } else if (id_normal == Obj_0_Status_ID_) {
      IsObjectFrameRecv.store(true, std::memory_order_relaxed);

      obj_status_recv_time_ = updateObjStatusRecvTS();
      ResetRadarObjectData(radar_object_list_, obj_status_recv_time_);

      radar_object_list_.num_of_object = HandleObjectStatus(candata->data);
      radar_object_index_ = 0;

    } else if (id_normal == Obj_1_General_ID_) {
      IsObjectFrameRecv.store(true, std::memory_order_relaxed);

      HandleObjectGeneral(candata->data, wait_time_object_status_,
                          obj_status_recv_time_, radar_object_index_,
                          radar_object_list_);

      if (radar_object_index_ <= 1) {
        valid_obj_ID_.clear();
      }
    } else if (id_normal == Obj_2_Quality_ID_) {
      IsObjectFrameRecv.store(true, std::memory_order_relaxed);

      if (radar_config_status_.SendQualityCfg) {
        GenerateValidObjID(radar_object_list_.Obj_ID, radar_object_index_,
                           valid_obj_ID_);
        HandleObjectQuality(candata->data, valid_obj_ID_, radar_object_list_);
      }

    } else if (id_normal == Obj_3_Extended_ID_) {
      IsObjectFrameRecv.store(true, std::memory_order_relaxed);

      if (radar_config_status_.SendExtInfoCfg) {
        GenerateValidObjID(radar_object_list_.Obj_ID, radar_object_index_,
                           valid_obj_ID_);
        HandleObjectExtended(candata->data, valid_obj_ID_, radar_object_list_);
      }

    } else if (id_normal == Obj_4_Warning_ID_) {
      // TODO: handle warning
      IsObjectFrameRecv.store(true, std::memory_order_relaxed);
    }

    return *this;
  }  // ParseRadarData

  // send speed info to Radar
  ContiRadarObject_600 &SpeedInputRadar(struct can_frame *candata,
                                        const float radar_speed) {
    RadarSpeed_Input_ = HandleSpeedInfo(candata, radar_speed);
    return *this;
  }  // SpeedInputRadar

  // send yaw rate info to Radar
  ContiRadarObject_600 &YawrateInputRadar(struct can_frame *candata,
                                          const float radar_yawrate) {
    RadarYawRate_Input_ = HandleYawRateInfo(candata, radar_yawrate);
    return *this;
  }  // YawrateInputRadar

  ContiRadarObject_600 &UpdateDriverPtStart() {
    // we should call this function every 7 days
    driver_pt_start_ = std::chrono::steady_clock::now();
    return *this;
  }  // UpdateDriverPtStart

  auto GetDriverPtStart() const noexcept { return driver_pt_start_; }
  auto GetRadarObjectInfoARS() const noexcept { return radar_object_list_; }

 private:
  std::atomic<bool> IsObjectFrameRecv = false;

  // the timestamp when the driver starts
  std::chrono::steady_clock::time_point driver_pt_start_ =
      std::chrono::steady_clock::now();

  const double wait_time_object_status_;  // unit: ms

  // message ID
  uint16_t RadarState_ID_;          // output
  uint16_t SpeedInformation_ID_;    // input
  uint16_t YawRateInformation_ID_;  // input
  uint16_t Obj_0_Status_ID_;        // output
  uint16_t Obj_1_General_ID_;       // output
  uint16_t Obj_2_Quality_ID_;       // output
  uint16_t Obj_3_Extended_ID_;      // output
  uint16_t Obj_4_Warning_ID_;       // output

  // Receiver
  RadarObjectInfoARS<max_num_object> radar_object_list_;
  std::size_t radar_object_index_;

  // timestamp when driver receives object status info
  double obj_status_recv_time_;

  // the ID of valid objects, ready for quality and extended info
  std::vector<uint8_t> valid_obj_ID_;

  ContiRadarRTConfig radar_config_status_;

  // Sender
  RadarSpeedInputARS RadarSpeed_Input_;
  RadarYawRateInputARS RadarYawRate_Input_;

  // update the timestamp when driver receives ObjStatusInfo
  double updateObjStatusRecvTS() {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - driver_pt_start_)
        .count();
  }  // updateObjStatusRecvTS

  // Initialize MessageID based on the SensorID
  void InitMessageID(const uint8_t SenorID) {
    if (SenorID <= 0x07) {
      uint16_t _SenorID = static_cast<uint16_t>(SenorID) << 4;
      RadarState_ID_ |= _SenorID;
      SpeedInformation_ID_ |= _SenorID;
      YawRateInformation_ID_ |= _SenorID;
      Obj_0_Status_ID_ |= _SenorID;
      Obj_1_General_ID_ |= _SenorID;
      Obj_2_Quality_ID_ |= _SenorID;
      Obj_3_Extended_ID_ |= _SenorID;
      Obj_4_Warning_ID_ |= _SenorID;
    }  // end if

  }  // InitMessageID

  // reset all the object data
  void ResetRadarObjectData(
      RadarObjectInfoARS<max_num_object> &radar_object_list,
      const double recv_ts) {
    radar_object_list.Obj_timestamp.fill(recv_ts);
    radar_object_list.Obj_IsValid.fill(false);
    radar_object_list.Obj_ID.fill(0);
    radar_object_list.Obj_DistLong.fill(0);
    radar_object_list.Obj_DistLat.fill(0);
    radar_object_list.Obj_VrelLong.fill(0);
    radar_object_list.Obj_VrelLat.fill(0);
    radar_object_list.Obj_DynProp.fill(0);
    radar_object_list.Obj_RCS.fill(0);
    radar_object_list.Obj_DistLong_rms.fill(0);
    radar_object_list.Obj_DistLat_rms.fill(0);
    radar_object_list.Obj_VrelLong_rms.fill(0);
    radar_object_list.Obj_VrelLat_rms.fill(0);
    radar_object_list.Obj_ArelLong_rms.fill(0);
    radar_object_list.Obj_ArelLat_rms.fill(0);
    radar_object_list.Obj_Orientation_rms.fill(0);
    radar_object_list.Obj_MeasState.fill(0);
    radar_object_list.Obj_ProbOfExist.fill(0);
    radar_object_list.Obj_ArelLong.fill(0);
    radar_object_list.Obj_ArelLat.fill(0);
    radar_object_list.Obj_Class.fill(0);
    radar_object_list.Obj_OrientationAngel.fill(0);
    radar_object_list.Obj_Length.fill(0);
    radar_object_list.Obj_Width.fill(0);
    radar_object_list.Obj_CollDetRegionBitfield.fill(0);

  }  // ResetRadarObjectData

  // //
  // std::size_t RecomputeRadarObjectData(const long int status_recv_time,
  //                                      const std::size_t radar_object_index)
  //                                      {
  //   long int delta_ts =
  //   std::chrono::duration_cast<std::chrono::milliseconds>(
  //                           std::chrono::steady_clock::now() -
  //                           driver_pt_start_) .count() -
  //                       status_recv_time;
  //   if (delta_ts >= wait_time_object_status) {
  //     if (radar_object_index > max_num_object) return max_num_object;
  //     return radar_object_index;
  //   }
  //   return 0;
  // }  // RecomputeRadarObjectData

  void GenerateValidObjID(const std::array<uint8_t, max_num_object> &Obj_ID,
                          const std::size_t num_valid,
                          std::vector<uint8_t> &valid_obj_ID) {
    if (valid_obj_ID.size() == 0) {
      valid_obj_ID =
          std::vector<uint8_t>(Obj_ID.begin(), Obj_ID.begin() + num_valid);
    }
  }  // GenerateValidObjID

  // parse Object Status Info: 0x60A
  std::size_t HandleObjectStatus(const uint8_t *candata) {
    auto num_of_object =
        static_cast<std::size_t>(parse_Object_NofObjects(candata));
    if (num_of_object > max_num_object) num_of_object = max_num_object;
    return num_of_object;
  }  // HandleObjectStatus

  // parse Object General Info: 0x60B
  void HandleObjectGeneral(
      const uint8_t *candata, const double wait_time_object_status,
      double &status_recv_time, std::size_t &radar_obj_index,
      RadarObjectInfoARS<max_num_object> &radar_object_list) {
    double current_ts = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - driver_pt_start_)
                            .count();
    // check if the objectstatus info is missing, normally
    // the delta timestamp between GeneralInfo and StatusInfo is less 40 ms
    double delta_ts = current_ts - status_recv_time;
    if (delta_ts >= wait_time_object_status) {
      status_recv_time = current_ts;
      radar_obj_index = 0;
      printf("status missing!!!!!!!!!!!!!!!!\n");
    }

    // check the index
    if (radar_obj_index >= max_num_object) {
      radar_obj_index = max_num_object;
      return;
    }

    // assign the value
    radar_object_list.Obj_timestamp[radar_obj_index] = current_ts;
    radar_object_list.Obj_IsValid[radar_obj_index] = true;
    radar_object_list.Obj_ID[radar_obj_index] = parse_Object_ID(candata);
    radar_object_list.Obj_DistLong[radar_obj_index] =
        parse_Object_DistLong(candata);
    radar_object_list.Obj_DistLat[radar_obj_index] =
        parse_Object_DistLat(candata);
    radar_object_list.Obj_VrelLong[radar_obj_index] =
        parse_Object_VrelLong(candata);
    radar_object_list.Obj_VrelLat[radar_obj_index] =
        parse_Object_VrelLat(candata);
    radar_object_list.Obj_DynProp[radar_obj_index] =
        parse_Object_DynProp(candata);
    radar_object_list.Obj_RCS[radar_obj_index] = parse_Object_RCS(candata);
    radar_object_list.Obj_ProbOfExist[radar_obj_index] = 0x7;  // default value
    radar_object_list.Obj_Length[radar_obj_index] = 0.2;       // default value
    radar_object_list.Obj_Width[radar_obj_index] = 0.2;        // default value

    // increase the index
    radar_obj_index++;
  }  // HandleObjectGeneral

  // parse Object Quality Info: 0x60C
  void HandleObjectQuality(
      const uint8_t *candata, const std::vector<uint8_t> &Obj_ID_valid,
      RadarObjectInfoARS<max_num_object> &radar_object_list) {
    // find the matched index in the set of GeneralInfo
    uint8_t current_ID = parse_Object_ID(candata);
    auto it = std::find(Obj_ID_valid.begin(), Obj_ID_valid.end(), current_ID);
    if (it != Obj_ID_valid.end()) {
      std::size_t matched_index = std::distance(Obj_ID_valid.begin(), it);
      if (matched_index >= max_num_object) return;
      radar_object_list.Obj_DistLong_rms[matched_index] =
          parse_Obj_DistLong_rms(candata);
      radar_object_list.Obj_DistLat_rms[matched_index] =
          parse_Obj_DistLat_rms(candata);
      radar_object_list.Obj_VrelLong_rms[matched_index] =
          parse_Obj_VrelLong_rms(candata);
      radar_object_list.Obj_VrelLat_rms[matched_index] =
          parse_Obj_VrelLat_rms(candata);
      radar_object_list.Obj_ArelLong_rms[matched_index] =
          parse_Obj_ArelLong_rms(candata);
      radar_object_list.Obj_ArelLat_rms[matched_index] =
          parse_Obj_ArelLat_rms(candata);
      radar_object_list.Obj_Orientation_rms[matched_index] =
          parse_Obj_Orientation_rms(candata);
      radar_object_list.Obj_MeasState[matched_index] =
          parse_Obj_MeasState(candata);
      radar_object_list.Obj_ProbOfExist[matched_index] =
          parse_Obj_ProbOfExist(candata);
    }

  }  // HandleObjectQuality

  void HandleObjectExtended(
      const uint8_t *candata, const std::vector<uint8_t> &Obj_ID_valid,
      RadarObjectInfoARS<max_num_object> &radar_object_list) {
    // find the matched index in the set of GeneralInfo
    uint8_t current_ID = parse_Object_ID(candata);
    auto it = std::find(Obj_ID_valid.begin(), Obj_ID_valid.end(), current_ID);
    if (it != Obj_ID_valid.end()) {
      std::size_t matched_index = std::distance(Obj_ID_valid.begin(), it);
      if (matched_index >= max_num_object) return;
      radar_object_list.Obj_ArelLong[matched_index] =
          parse_Object_ArelLong(candata);
      radar_object_list.Obj_ArelLat[matched_index] =
          parse_Object_ArelLat(candata);
      radar_object_list.Obj_Class[matched_index] = parse_Object_Class(candata);
      radar_object_list.Obj_OrientationAngel[matched_index] =
          parse_Object_OrientationAngel(candata);
      radar_object_list.Obj_Length[matched_index] =
          parse_Object_Length(candata);
      radar_object_list.Obj_Width[matched_index] = parse_Object_Width(candata);
    }
  }  // HandleObjectExtended

  void HandleObjectCollison(
      const uint8_t *candata,
      RadarObjectInfoARS<max_num_object> &radar_object_list) {
    int radar_object_counter = 0;

    radar_object_list.Obj_CollDetRegionBitfield[radar_object_counter] =
        parse_Object_CollDetRegionBitfield(candata);

  }  // HandleObjectExtended

  ContiRadarRTConfig HandleConfigStatus(const uint8_t *candata) {
    ContiRadarRTConfig ContiRadar_RTConfig{
        false,  // NVMReadStatus
        0x0,    // SensorID
        0x0,    // RadarError
        false,  // SendQualityCfg
        false,  // SendExtInfoCfg
        0x0     // MotionRxState
    };

    if (parse_NVMread_status(candata) == 0x1)
      ContiRadar_RTConfig.NVMReadStatus = true;

    ContiRadar_RTConfig.SensorID = parse_sensor_ID(candata);

    if (parse_send_quality(candata) == 0x1)
      ContiRadar_RTConfig.SendQualityCfg = true;

    if (parse_send_ext_info(candata) == 0x1)
      ContiRadar_RTConfig.SendExtInfoCfg = true;

    ContiRadar_RTConfig.MotionRxState = parse_motion_rx_state(candata);

    uint8_t persistent_error = parse_persistent_error(candata);
    uint8_t interference = parse_interference(candata);
    uint8_t temperature_error = parse_temperature_error(candata);
    uint8_t temporary_error = parse_temporary_error(candata);
    uint8_t voltage_error = parse_voltage_error(candata);

    if (persistent_error == 0x1) ContiRadar_RTConfig.RadarError++;
    if (interference == 0x1) ContiRadar_RTConfig.RadarError++;
    if (temperature_error == 0x1) ContiRadar_RTConfig.RadarError++;
    if (temporary_error == 0x1) ContiRadar_RTConfig.RadarError++;
    if (voltage_error == 0x1) ContiRadar_RTConfig.RadarError++;

    return ContiRadar_RTConfig;

  }  // HandleConfigStatus

  RadarSpeedInputARS HandleSpeedInfo(struct can_frame *candata,
                                     const float radar_speed) {
    RadarSpeedInputARS RadarSpeed_Input{
        0,  // Radar_SpeedDirection
        0   // Radar_SpeedValue
    };

    float radar_speed_abs = std::fabs(radar_speed);

    uint8_t speed_direction = 0;
    if (radar_speed_abs < 0.02) {
      speed_direction = 0x0;  // stantstill
    } else if (radar_speed < 0) {
      speed_direction = 0x2;  // backward
    } else {
      speed_direction = 0x1;  // forward
    }

    uint16_t speed_value = static_cast<uint16_t>(radar_speed_abs / 0.02);

    candata->can_id = SpeedInformation_ID_;
    candata->can_dlc = 2;
    candata->data[0] = ((speed_direction << 6) & 0xC0) |
                       static_cast<uint8_t>((speed_value & 0x1F00) >> 8);
    candata->data[1] = static_cast<uint8_t>(speed_value & 0x00FF);

    RadarSpeed_Input.Radar_SpeedDirection = speed_direction;
    RadarSpeed_Input.Radar_SpeedValue = speed_value;

    return RadarSpeed_Input;

  }  // HandleSpeedInfo

  RadarYawRateInputARS HandleYawRateInfo(struct can_frame *candata,
                                         const float radar_yawrate) {
    RadarYawRateInputARS RadarYawRate_Input{
        0  // Radar_YawRateValue
    };

    // Due to radar 408 manual: max 327.68, res 0.01, unit:deg/s
    uint16_t yaw_rate_value =
        static_cast<uint16_t>((radar_yawrate + 327.68) * 100);

    candata->can_id = YawRateInformation_ID_;
    candata->can_dlc = 2;
    candata->data[0] = static_cast<uint8_t>((yaw_rate_value & 0xFF00) >> 8);
    candata->data[1] = static_cast<uint8_t>(yaw_rate_value & 0x00FF);

    RadarYawRate_Input.Radar_YawRateValue = yaw_rate_value;

    return RadarYawRate_Input;

  }  // HandleYawRateInfo

  /*************************  0x60A  ***************************/
  uint8_t parse_Object_NofObjects(const uint8_t *bytes) const {
    return bytes[0];
  }  // parse_Object_NofObjects
  uint16_t parse_Object_MeasCounter(const uint8_t *bytes) const {
    return (static_cast<uint16_t>(bytes[1]) << 8) |
           static_cast<uint16_t>(bytes[2]);
  }  // parse_Object_MeasCounter
  uint8_t parse_Object_InterfaceVersion(const uint8_t *bytes) const {
    return (bytes[3] >> 4);
  }  // parse_Object_InterfaceVersion

  /*************************  0x60B  ***************************/
  uint8_t parse_Object_ID(const uint8_t *bytes) const {
    return bytes[0];
  }  // parse_Object_ID
  float parse_Object_DistLong(const uint8_t *bytes) const {
    uint16_t value = (static_cast<uint16_t>(bytes[1]) << 5) |
                     static_cast<uint16_t>(bytes[2] >> 3);
    return 0.2 * value - 500.0;
  }  // parse_Object_DistLong
  float parse_Object_DistLat(const uint8_t *bytes) const {
    uint16_t value = (static_cast<uint16_t>(bytes[2] & 0b111) << 8) |
                     static_cast<uint16_t>(bytes[3]);
    return 0.2 * value - 204.6;
  }  // parse_Object_DistLat
  float parse_Object_VrelLong(const uint8_t *bytes) const {
    uint16_t value = (static_cast<uint16_t>(bytes[4]) << 2) |
                     static_cast<uint16_t>(bytes[5] >> 6);
    return 0.25 * value - 128.0;
  }  // parse_Object_VrelLong
  float parse_Object_VrelLat(const uint8_t *bytes) const {
    uint16_t value = (static_cast<uint16_t>(bytes[5] & 0b111111) << 3) |
                     static_cast<uint16_t>(bytes[6] >> 5);
    return 0.25 * value - 64.0;
  }  // parse_Object_VrelLat
  uint8_t parse_Object_DynProp(const uint8_t *bytes) const {
    return (bytes[6] & 0b111);
  }  // parse_Object_DynProp
  float parse_Object_RCS(const uint8_t *bytes) const {
    return 0.5 * bytes[7] - 64.0;
  }  // parse_Object_RCS

  /*************************  0x60C  ***************************/
  uint8_t parse_Obj_DistLong_rms(const uint8_t *bytes) const {
    return (bytes[1] >> 3);
  }  // parse_Obj_DistLong_rms
  uint8_t parse_Obj_VrelLong_rms(const uint8_t *bytes) const {
    return ((bytes[2] & 0b111110) >> 1);
  }  // parse_Obj_VrelLong_rms
  uint8_t parse_Obj_DistLat_rms(const uint8_t *bytes) const {
    return ((bytes[1] & 0b111) << 2) | (bytes[2] >> 6);
  }  // parse_Obj_DistLat_rms
  uint8_t parse_Obj_VrelLat_rms(const uint8_t *bytes) const {
    return ((bytes[2] & 0b1) << 4) | (bytes[3] >> 4);
  }  // parse_Obj_VrelLat_rms
  uint8_t parse_Obj_ArelLat_rms(const uint8_t *bytes) const {
    return ((bytes[4] & 0b1111100) >> 2);
  }  // parse_Obj_ArelLat_rms
  uint8_t parse_Obj_ArelLong_rms(const uint8_t *bytes) const {
    return ((bytes[3] & 0b1111) << 1) | (bytes[4] >> 7);
  }  // parse_Obj_ArelLong_rms
  uint8_t parse_Obj_Orientation_rms(const uint8_t *bytes) const {
    return ((bytes[4] & 0b11) << 3) | (bytes[5] >> 5);
  }  // parse_Obj_Orientation_rms
  uint8_t parse_Obj_MeasState(const uint8_t *bytes) const {
    return ((bytes[6] & 0b11100) >> 2);
  }  // parse_Obj_MeasState
  uint8_t parse_Obj_ProbOfExist(const uint8_t *bytes) const {
    return (bytes[6] >> 5);
  }  // parse_Obj_ProbOfExist

  /*************************  0x60D  ***************************/
  float parse_Object_ArelLong(const uint8_t *bytes) const {
    uint16_t value = (static_cast<uint16_t>(bytes[1]) << 3) |
                     static_cast<uint16_t>(bytes[2] >> 5);
    return 0.01 * value - 10.0;
  }  // parse_Object_ArelLong
  uint8_t parse_Object_Class(const uint8_t *bytes) const {
    return (bytes[3] & 0b111);
  }  // parse_Object_Class
  float parse_Object_ArelLat(const uint8_t *bytes) const {
    uint16_t value = (static_cast<uint16_t>(bytes[2] & 0b11111) << 4) |
                     static_cast<uint16_t>(bytes[3] >> 4);
    return 0.01 * value - 2.50;
  }  // parse_Object_ArelLat
  float parse_Object_OrientationAngel(const uint8_t *bytes) const {
    uint16_t value = (static_cast<uint16_t>(bytes[4]) << 2) |
                     static_cast<uint16_t>(bytes[5] >> 6);
    return 0.4 * value - 180.0;
  }  // parse_Object_OrientationAngel
  float parse_Object_Length(const uint8_t *bytes) const {
    return 0.2 * bytes[6];
  }  // parse_Object_Length
  float parse_Object_Width(const uint8_t *bytes) const {
    return 0.2 * bytes[7];
  }  // parse_Object_Width

  /*************************  0x60E  ***************************/
  uint8_t parse_Object_CollDetRegionBitfield(const uint8_t *bytes) const {
    return bytes[1];
  }  // parse_Object_CollDetRegionBitfield

  /*************************  0x201  ***************************/
  uint8_t parse_NVMread_status(const uint8_t *bytes) const {
    return ((bytes[0] & 0b1000000) >> 6);
  }  // parse_NVMread_status
  uint8_t parse_send_quality(const uint8_t *bytes) const {
    return ((bytes[5] & 0b10000) >> 4);
  }  // parse_send_quality
  uint8_t parse_send_ext_info(const uint8_t *bytes) const {
    return ((bytes[5] & 0b100000) >> 5);
  }  // parse_send_ext_info
  uint8_t parse_sensor_ID(const uint8_t *bytes) const {
    return (bytes[4] & 0b111);
  }  // parse_sensor_ID
  uint8_t parse_motion_rx_state(const uint8_t *bytes) const {
    return (bytes[5] >> 6);
  }  // parse_motion_rx_state
  uint8_t parse_persistent_error(const uint8_t *bytes) const {
    return ((bytes[2] & 0b100000) >> 5);
  }  // parse_persistent_error
  uint8_t parse_interference(const uint8_t *bytes) const {
    return ((bytes[2] & 0b10000) >> 4);
  }  // parse_Interference
  uint8_t parse_temperature_error(const uint8_t *bytes) const {
    return ((bytes[2] & 0b1000) >> 3);
  }  // parse_temperature_error
  uint8_t parse_temporary_error(const uint8_t *bytes) const {
    return ((bytes[2] & 0b100) >> 2);
  }  // parse_temporary_error
  uint8_t parse_voltage_error(const uint8_t *bytes) const {
    return ((bytes[2] & 0b10) >> 1);
  }  // parse_voltage_error

  /*************************  0x408  ***************************/
  uint8_t parse_CollDetState_Activation(const uint8_t *bytes) const {
    return ((bytes[0] & 0b10) >> 1);
  }  // parse_CollDetState_Activation
  uint8_t parse_CollDetState_NofRegions(const uint8_t *bytes) const {
    return (bytes[0] >> 4);
  }  // parse_CollDetState_NofRegions
  float parse_CollDetState_MinDetectTime(const uint8_t *bytes) const {
    return 0.1 * bytes[1];
  }  // parse_CollDetState_MinDetectTime
  uint16_t parse_CollDetState_MeasCounter(const uint8_t *bytes) const {
    return (static_cast<uint16_t>(bytes[2]) << 8) |
           static_cast<uint16_t>(bytes[3]);
  }  // parse_CollDetState_MeasCounter

};  // end class

#endif /* _CONTIRADAROBJECT_600_H_ */