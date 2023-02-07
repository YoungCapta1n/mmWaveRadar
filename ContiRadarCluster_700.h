/*
****************************************************************************
* ContiRadarCluster_700.h:
* Receive Data from Continental mmWaveRadar (ARS40X), etc.
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#ifndef _CONTIRADARCLUSTER_700_H_
#define _CONTIRADARCLUSTER_700_H_

#include <linux/can.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>

#include "ContiRadarData.h"

// The driver of ARS40X Radar
template <uint32_t max_num_cluster>
class ContiRadarCluster_700 {
  enum class RadarClusterStatus {
    IDLE = 0,
    GENERAL_INFO,  //
    QUALITY_INFO   //
  };
  struct RadarSpeedInputARS {
    uint8_t Radar_SpeedDirection;
    uint16_t Radar_SpeedValue;
  };  // RadarSpeedInputARS
  struct RadarYawRateInputARS {
    uint16_t Radar_YawRateValue;
  };  // RadarYawRateInputARS

 public:
  ContiRadarCluster_700(const uint8_t sensorID) { InitMessageID(sensorID); }
  virtual ~ContiRadarCluster_700() = default;

  // parse radar data from can frame
  ContiRadarCluster_700 &ParseRadarData(const struct can_frame *candata) {
    canid_t id_normal = candata->can_id;

    if (id_normal == RadarState_ID_) {
      radar_config_status_ = HandleConfigStatus(candata);
    } else if ((id_normal == Cluster_0_Status_ID_) ||
               (id_normal == Cluster_1_General_ID_) ||
               (id_normal == Cluster_2_Quality_ID_)) {
      HandleClusterData(id_normal, candata, radar_config_status_);
    }
    return *this;
  }  // ParseRadarData

  // send speed info to Radar
  ContiRadarCluster_700 &SpeedInputRadar(struct can_frame *candata,
                                         const float radar_speed) {
    RadarSpeed_Input_ = HandleSpeedInfo(candata, radar_speed);
    return *this;
  }  // SpeedInputRadar

  // send yaw rate info to Radar
  ContiRadarCluster_700 &YawrateInputRadar(struct can_frame *candata,
                                           const float radar_yawrate) {
    RadarYawRate_Input_ = HandleYawRateInfo(candata, radar_yawrate);
    return *this;
  }  // YawrateInputRadar

  auto GetDriverPtStart() const noexcept { return driver_pt_start_; }
  auto GetNumofCluster() const noexcept { return num_of_cluster_; }
  RadarClusterInfoARS *GetRadarClusterInfoARS() noexcept {
    return radar_cluster_list_;
  }

 private:
  // message ID
  uint16_t RadarState_ID_ = 0x201;          // output
  uint16_t SpeedInformation_ID_ = 0x300;    // input
  uint16_t YawRateInformation_ID_ = 0x301;  // input
  uint16_t Cluster_0_Status_ID_ = 0x600;    // output
  uint16_t Cluster_1_General_ID_ = 0x701;   // output
  uint16_t Cluster_2_Quality_ID_ = 0x702;   // output

  RadarClusterInfoARS radar_cluster_list_[max_num_cluster];
  uint32_t num_of_cluster_near_ = 0;
  uint32_t num_of_cluster_far_ = 0;
  uint32_t num_of_cluster_ = 0;
  uint32_t radar_cluster_counter_ = 0;
  RadarClusterStatus radar_cluster_receive_status_ = RadarClusterStatus::IDLE;

  ContiRadarRTConfig radar_config_status_{
      false,  // NVMReadStatus
      0x0,    // SensorID
      0x0,    // RadarError
      false,  // SendQualityCfg
      false,  // SendExtInfoCfg
      0x0     // MotionRxState
  };

  std::chrono::steady_clock::time_point driver_pt_start_ =
      std::chrono::steady_clock::now();

  // Sender
  RadarSpeedInputARS RadarSpeed_Input_{
      0,  // Radar_SpeedDirection
      0   // Radar_SpeedValue
  };
  RadarYawRateInputARS RadarYawRate_Input_{
      0  // Radar_YawRateValue
  };

  void InitMessageID(const uint8_t SenorID) {
    if (SenorID <= 0x07) {
      uint16_t _SenorID = static_cast<uint16_t>(SenorID) << 4;
      RadarState_ID_ |= _SenorID;
      SpeedInformation_ID_ |= _SenorID;
      YawRateInformation_ID_ |= _SenorID;
      Cluster_0_Status_ID_ |= _SenorID;
      Cluster_1_General_ID_ |= _SenorID;
      Cluster_2_Quality_ID_ |= _SenorID;
    }  // end if

  }  // InitMessageID

  void HandleClusterData(const canid_t id, const struct can_frame *candata,
                         const ContiRadarRTConfig &ContiRadar_RTConfig) {
    switch (radar_cluster_receive_status_) {
      case RadarClusterStatus::IDLE: {
        if (id == Cluster_0_Status_ID_) {
          num_of_cluster_near_ = static_cast<uint32_t>(
              parse_Cluster_NofClustersNear(candata->data));
          num_of_cluster_far_ = static_cast<uint32_t>(
              parse_Cluster_NofClustersFar(candata->data));
          num_of_cluster_ = num_of_cluster_near_ + num_of_cluster_far_;

          if (num_of_cluster_ > max_num_cluster)
            num_of_cluster_ = max_num_cluster;
          radar_cluster_counter_ = 0;

          if (num_of_cluster_ == 0) {
            radar_cluster_receive_status_ = RadarClusterStatus::IDLE;
          } else {
            radar_cluster_receive_status_ = RadarClusterStatus::GENERAL_INFO;
          }
        }
      } break;

      case RadarClusterStatus::GENERAL_INFO: {
        if (id == Cluster_0_Status_ID_) {
          num_of_cluster_near_ = static_cast<uint32_t>(
              parse_Cluster_NofClustersNear(candata->data));
          num_of_cluster_far_ = static_cast<uint32_t>(
              parse_Cluster_NofClustersFar(candata->data));
          num_of_cluster_ = num_of_cluster_near_ + num_of_cluster_far_;

          if (num_of_cluster_ > max_num_cluster)
            num_of_cluster_ = max_num_cluster;
          radar_cluster_counter_ = 0;

          if (num_of_cluster_ == 0) {
            radar_cluster_receive_status_ = RadarClusterStatus::IDLE;
          } else {
            radar_cluster_receive_status_ = RadarClusterStatus::GENERAL_INFO;
          }
        } else if (id == Cluster_1_General_ID_) {
          radar_cluster_list_[radar_cluster_counter_].Cluster_timestamp =
              std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::steady_clock::now() - driver_pt_start_)
                  .count();
          radar_cluster_list_[radar_cluster_counter_].Cluster_ID =
              static_cast<uint32_t>(parse_Cluster_ID(candata->data));
          radar_cluster_list_[radar_cluster_counter_].Cluster_DistLong =
              parse_Cluster_DistLong(candata->data);
          radar_cluster_list_[radar_cluster_counter_].Cluster_DistLat =
              parse_Cluster_DistLat(candata->data);
          radar_cluster_list_[radar_cluster_counter_].Cluster_VrelLong =
              parse_Cluster_VrelLong(candata->data);
          radar_cluster_list_[radar_cluster_counter_].Cluster_VrelLat =
              parse_Cluster_VrelLat(candata->data);
          radar_cluster_list_[radar_cluster_counter_].Cluster_DynProp =
              parse_Cluster_DynProp(candata->data);
          radar_cluster_list_[radar_cluster_counter_].Cluster_RCS =
              parse_Cluster_RCS(candata->data);

          radar_cluster_counter_ += 1;

          if (radar_cluster_counter_ >= num_of_cluster_) {
            if (ContiRadar_RTConfig.SendQualityCfg) {
              radar_cluster_receive_status_ = RadarClusterStatus::QUALITY_INFO;
              radar_cluster_counter_ = 0;
            } else {
              radar_cluster_receive_status_ = RadarClusterStatus::IDLE;
              radar_cluster_counter_ = 0;
            }
          }
        }
      } break;

      case RadarClusterStatus::QUALITY_INFO: {
        if (id == Cluster_0_Status_ID_) {
          num_of_cluster_near_ = static_cast<uint32_t>(
              parse_Cluster_NofClustersNear(candata->data));
          num_of_cluster_far_ = static_cast<uint32_t>(
              parse_Cluster_NofClustersFar(candata->data));
          num_of_cluster_ = num_of_cluster_near_ + num_of_cluster_far_;

          if (num_of_cluster_ > max_num_cluster)
            num_of_cluster_ = max_num_cluster;
          radar_cluster_counter_ = 0;

          if (num_of_cluster_ == 0) {
            radar_cluster_receive_status_ = RadarClusterStatus::IDLE;
          } else {
            radar_cluster_receive_status_ = RadarClusterStatus::GENERAL_INFO;
          }
        } else if (id == Cluster_2_Quality_ID_) {
          radar_cluster_list_[radar_cluster_counter_].Cluster_DistLong_rms =
              parse_Cluster_DistLong_rms(candata->data);
          radar_cluster_list_[radar_cluster_counter_].Cluster_DistLat_rms =
              parse_Cluster_DistLat_rms(candata->data);
          radar_cluster_list_[radar_cluster_counter_].Cluster_VrelLong_rms =
              parse_Cluster_VrelLong_rms(candata->data);
          radar_cluster_list_[radar_cluster_counter_].Cluster_VrelLat_rms =
              parse_Cluster_VrelLat_rms(candata->data);
          radar_cluster_list_[radar_cluster_counter_].Cluster_Pdh0 =
              parse_Cluster_Pdh0(candata->data);
          radar_cluster_list_[radar_cluster_counter_].Cluster_AmbigState =
              parse_Cluster_AmbigState(candata->data);
          radar_cluster_list_[radar_cluster_counter_].Cluster_InvalidState =
              parse_Cluster_InvalidState(candata->data);

          radar_cluster_counter_ += 1;

          if (radar_cluster_counter_ >= num_of_cluster_) {
            radar_cluster_receive_status_ = RadarClusterStatus::IDLE;
            radar_cluster_counter_ = 0;
          }
        }
      } break;

      default:
        radar_cluster_receive_status_ = RadarClusterStatus::IDLE;
        break;
    }
  }  // HandleClusterData

  ContiRadarRTConfig HandleConfigStatus(const struct can_frame *candata) {
    ContiRadarRTConfig ContiRadar_RTConfig{
        false,  // NVMReadStatus
        0x0,    // SensorID
        0x0,    // RadarError
        false,  // SendQualityCfg
        false,  // SendExtInfoCfg
        0x0     // MotionRxState
    };

    if (parse_NVMread_status(candata->data) == 0x1)
      ContiRadar_RTConfig.NVMReadStatus = true;

    ContiRadar_RTConfig.SensorID = parse_sensor_ID(candata->data);

    if (parse_send_quality(candata->data) == 0x1)
      ContiRadar_RTConfig.SendQualityCfg = true;

    if (parse_send_ext_info(candata->data) == 0x1)
      ContiRadar_RTConfig.SendExtInfoCfg = true;

    ContiRadar_RTConfig.MotionRxState = parse_motion_rx_state(candata->data);

    uint8_t persistent_error = parse_persistent_error(candata->data);
    uint8_t interference = parse_interference(candata->data);
    uint8_t temperature_error = parse_temperature_error(candata->data);
    uint8_t temporary_error = parse_temporary_error(candata->data);
    uint8_t voltage_error = parse_voltage_error(candata->data);

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

  /*************************  0x600  ***************************/
  uint8_t parse_Cluster_NofClustersNear(const uint8_t *bytes) const {
    return bytes[0];
  }  // parse_Cluster_NofClustersNear
  uint8_t parse_Cluster_NofClustersFar(const uint8_t *bytes) const {
    return bytes[1];
  }  // parse_Cluster_NofClustersFar
  uint16_t parse_Cluster_MeasCounter(const uint8_t *bytes) const {
    return (static_cast<uint16_t>(bytes[2]) << 8) |
           static_cast<uint16_t>(bytes[3]);
  }  // parse_Cluster_MeasCounter
  uint8_t parse_Cluster_InterfaceVersion(const uint8_t *bytes) const {
    return bytes[4] >> 4;
  }  // parse_Cluster_InterfaceVersion

  /*************************  0x701  ***************************/
  uint8_t parse_Cluster_ID(const uint8_t *bytes) const {
    return bytes[0];
  }  // parse_Cluster_ID
  float parse_Cluster_DistLong(const uint8_t *bytes) const {
    uint16_t value = (static_cast<uint16_t>(bytes[1]) << 5) |
                     static_cast<uint16_t>(bytes[2] >> 3);
    return 0.2 * value - 500.0;
  }  // parse_Cluster_DistLong
  float parse_Cluster_DistLat(const uint8_t *bytes) const {
    uint16_t value = (static_cast<uint16_t>(bytes[2] & 0b11) << 8) |
                     static_cast<uint16_t>(bytes[3]);
    return 0.2 * value - 102.3;
  }  // parse_Cluster_DistLat
  float parse_Cluster_VrelLong(const uint8_t *bytes) const {
    uint16_t value = (static_cast<uint16_t>(bytes[4]) << 2) |
                     static_cast<uint16_t>(bytes[5] >> 6);
    return 0.25 * value - 128.0;
  }  // parse_Cluster_VrelLong
  uint8_t parse_Cluster_DynProp(const uint8_t *bytes) const {
    return bytes[6] & 0b111;
  }  // parse_Cluster_DynProp
  float parse_Cluster_VrelLat(const uint8_t *bytes) const {
    uint16_t value = (static_cast<uint16_t>(bytes[5] & 0b111111) << 3) |
                     static_cast<uint16_t>(bytes[6] >> 5);
    return 0.25 * value - 64.0;
  }  // parse_Cluster_VrelLat
  float parse_Cluster_RCS(const uint8_t *bytes) const {
    return 0.5 * bytes[7] - 64.0;
  }  // parse_Cluster_RCS

  /*************************  0x702  ***************************/
  uint8_t parse_Cluster_DistLong_rms(const uint8_t *bytes) const {
    return bytes[1] >> 3;
  }  // parse_Cluster_DistLong_rms
  uint8_t parse_Cluster_VrelLong_rms(const uint8_t *bytes) const {
    return (bytes[2] & 0b111110) >> 1;
  }  // parse_Cluster_VrelLong_rms
  uint8_t parse_Cluster_DistLat_rms(const uint8_t *bytes) const {
    return ((bytes[1] & 0b111) << 2) | (bytes[2] >> 6);
  }  // parse_Cluster_DistLat_rms
  uint8_t parse_Cluster_Pdh0(const uint8_t *bytes) const {
    return bytes[3] & 0b111;
  }  // parse_Cluster_Pdh0
  uint8_t parse_Cluster_VrelLat_rms(const uint8_t *bytes) const {
    return ((bytes[2] & 0b1) << 4) | (bytes[3] >> 4);
  }  // parse_Cluster_VrelLat_rms
  uint8_t parse_Cluster_AmbigState(const uint8_t *bytes) const {
    return bytes[4] & 0b111;
  }  // parse_Cluster_AmbigState
  uint8_t parse_Cluster_InvalidState(const uint8_t *bytes) const {
    return bytes[4] >> 3;
  }  // parse_Cluster_InvalidState

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

};  // end class ContiRadarCluster_700

#endif /* _CONTIRADARCLUSTER_700_H_ */