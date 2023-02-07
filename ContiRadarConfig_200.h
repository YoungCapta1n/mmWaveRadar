/*
****************************************************************************
* ContiRadarConfig_200.h:
* Receive Data from Continental mmWaveRadar (ARS40X), etc.
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#ifndef _ContiRadarConfig_200_H_
#define _ContiRadarConfig_200_H_

#include <linux/can.h>

#include "ContiRadarData.h"

/************** RadarType ****************/
// 0: ARS408-21
// 1: ARS404-21
// 2: SRR308-21
template <int RadarType>
class ContiRadarConfig200 {
 public:
  ContiRadarConfig200()
      : previous_sensor_ID_(0x0),
        Read_Previous_Radar_Config_({
            0x0,  // SensorID
            200,  // MaxDistance
            0x0,  // RadarPower
            0x0,  // OutputType
            0x0,  // SendQuality
            0x0,  // SendExtInfo
            0x0,  // SortIndex
            0x0,  // CtrlRelay
            0x0   // RCS_Threshold
        }),
        Input_ContiRadar_Config_({
            0x0,  // SensorID
            200,  // MaxDistance
            0x0,  // RadarPower
            0x0,  // OutputType
            0x0,  // SendQuality
            0x0,  // SendExtInfo
            0x0,  // SortIndex
            0x0,  // CtrlRelay
            0x0   // RCS_Threshold
        }),
        Read_Current_Radar_Config_({
            0x0,  // SensorID
            200,  // MaxDistance
            0x0,  // RadarPower
            0x0,  // OutputType
            0x0,  // SendQuality
            0x0,  // SendExtInfo
            0x0,  // SortIndex
            0x0,  // CtrlRelay
            0x0   // RCS_Threshold
        }) {}
  ~ContiRadarConfig200() {}

  std::string MapRadarType2String() {
    std::string radartype = "ARS408-21";
    switch (RadarType) {
      case 0:
        radartype = "ARS408-21";
        break;
      case 1:
        radartype = "ARS404-21";
        break;
      case 2:
        radartype = "SRR308-21";
        break;
      default:
        break;
    }
    return radartype;
  }  // MapRadarType2String

  // check if the config is successfully setup
  int CheckCurrentConfigStatus(const struct can_frame* candata) {
    uint32_t Input_RadarState_ID =
        (0x201 |
         (static_cast<uint32_t>(Input_ContiRadar_Config_.SensorID) << 4));

    if (candata->can_id == Input_RadarState_ID) {
      uint8_t NVMReadStatus = parse_NVMread_status(candata->data);

      if (NVMReadStatus == 0x1) {
        uint8_t sensor_ID = parse_sensor_ID(candata->data);
        Read_Current_Radar_Config_.SensorID = sensor_ID;
        if (Input_ContiRadar_Config_.SensorID != sensor_ID) {
          printf("Error in parse_sensor_ID\n");
          return 0;
        }

        switch (RadarType) {
          case 0: {
            uint16_t max_dist = parse_max_dist(candata->data);
            Read_Current_Radar_Config_.MaxDistance = max_dist;
            if (Input_ContiRadar_Config_.MaxDistance != max_dist) {
              printf("Error in parse_max_dist\n");
              return 0;
            }
          } break;
          case 1: {
            uint16_t max_dist = parse_max_dist(candata->data);
            Read_Current_Radar_Config_.MaxDistance = max_dist;
            if (190 != max_dist) {
              printf("Error in parse_max_dist\n");
              return 0;
            }
          } break;
          case 2: {
            uint16_t max_dist = parse_max_dist(candata->data);
            Read_Current_Radar_Config_.MaxDistance = max_dist;
            if (96 != max_dist) {
              printf("Error in parse_max_dist\n");
              return 0;
            }
          } break;
          default:
            break;
        }  // end switch

        uint8_t radar_power = parse_radar_power(candata->data);
        Read_Current_Radar_Config_.RadarPower = radar_power;
        if (Input_ContiRadar_Config_.RadarPower != radar_power) {
          printf("Error in parse_radar_power\n");
          return 0;
        }

        uint8_t output_type = parse_output_type(candata->data);
        Read_Current_Radar_Config_.OutputType = output_type;
        if (Input_ContiRadar_Config_.OutputType != output_type) {
          printf("Error in parse_output_type\n");
          return 0;
        }

        uint8_t send_quality = parse_send_quality(candata->data);
        Read_Current_Radar_Config_.SendQuality = send_quality;
        if (Input_ContiRadar_Config_.SendQuality != send_quality) {
          printf("Error in parse_send_quality\n");
          return 0;
        }

        uint8_t send_ext_info = parse_send_ext_info(candata->data);
        Read_Current_Radar_Config_.SendExtInfo = send_ext_info;
        if (Input_ContiRadar_Config_.SendExtInfo != send_ext_info) {
          printf("Error in parse_send_ext_info\n");
          return 0;
        }

        uint8_t sort_index = parse_sort_index(candata->data);
        Read_Current_Radar_Config_.SortIndex = sort_index;
        if (Input_ContiRadar_Config_.SortIndex != sort_index) {
          printf("Error in parse_sort_index\n");
          return 0;
        }

        uint8_t rcs_threshold = parse_rcs_threshold(candata->data);
        Read_Current_Radar_Config_.RCS_Threshold = rcs_threshold;
        if (Input_ContiRadar_Config_.RCS_Threshold != rcs_threshold) {
          printf("Error in parse_rcs_threshold\n");
          return 0;
        }

        uint8_t ctrl_relay = parse_ctrl_relay(candata->data);
        Read_Current_Radar_Config_.CtrlRelay = ctrl_relay;
        if (Input_ContiRadar_Config_.CtrlRelay != ctrl_relay) {
          printf("Error in parse_ctrl_relay\n");
          return 0;
        }

        return 1;
      }
      printf("Error in parse_NVMread_status\n");
      return 0;

    }  // end if
    return -1;

  }  // CheckCurrentConfigStatus

  bool ParsePreviousRadarConfig(const struct can_frame* candata) {
    // parse the sensor_id
    uint32_t cresult = static_cast<uint32_t>(candata->can_id);

    uint8_t high = static_cast<uint8_t>((cresult & 0x0F00) >> 8);
    uint8_t middle = static_cast<uint8_t>((cresult & 0xF0) >> 4);
    uint8_t low = static_cast<uint8_t>(cresult & 0x0F);

    if ((high == 0x2) && (low == 0x1)) {
      //
      previous_sensor_ID_ = middle;

      uint8_t NVMReadStatus = parse_NVMread_status(candata->data);
      if (NVMReadStatus == 0x1) {
        Read_Previous_Radar_Config_.SensorID = parse_sensor_ID(candata->data);
        Read_Previous_Radar_Config_.MaxDistance = parse_max_dist(candata->data);
        Read_Previous_Radar_Config_.RadarPower =
            parse_radar_power(candata->data);
        Read_Previous_Radar_Config_.OutputType =
            parse_output_type(candata->data);
        Read_Previous_Radar_Config_.SendQuality =
            parse_send_quality(candata->data);
        Read_Previous_Radar_Config_.SendExtInfo =
            parse_send_ext_info(candata->data);
        Read_Previous_Radar_Config_.SortIndex = parse_sort_index(candata->data);
        Read_Previous_Radar_Config_.RCS_Threshold =
            parse_rcs_threshold(candata->data);
        Read_Previous_Radar_Config_.CtrlRelay = parse_ctrl_relay(candata->data);

        return true;
      }  // end if
    }

    return false;
  }  // ParsePreviousRadarConfig

  // update the radar config
  ContiRadarConfig200& SetupInputContiRadarConfig(
      const uint8_t new_sensor_ID, const uint16_t max_distance,
      const uint8_t output_type, const bool IsOutputObjectQuality,
      const bool IsOutputExtInfo, const uint8_t SortIndexType = 0x0,
      const uint8_t RadarPower = 0x0, const uint8_t RCS_Threshold = 0x0,
      const uint8_t RelayControl = 0x0) {
    if (new_sensor_ID <= 0x07) {
      Input_ContiRadar_Config_.SensorID = new_sensor_ID;
    } else
      Input_ContiRadar_Config_.SensorID = 0x0;

    Input_ContiRadar_Config_.MaxDistance = max_distance;
    Input_ContiRadar_Config_.OutputType = output_type;

    if (IsOutputObjectQuality)
      Input_ContiRadar_Config_.SendQuality = 0x1;
    else
      Input_ContiRadar_Config_.SendQuality = 0x0;
    if (IsOutputExtInfo)
      Input_ContiRadar_Config_.SendExtInfo = 0x1;
    else
      Input_ContiRadar_Config_.SendExtInfo = 0x0;

    Input_ContiRadar_Config_.SortIndex = SortIndexType;
    Input_ContiRadar_Config_.RadarPower = RadarPower;
    Input_ContiRadar_Config_.CtrlRelay = RelayControl;
    Input_ContiRadar_Config_.RCS_Threshold = RCS_Threshold;

    return *this;

  }  // SetupInputContiRadarConfig

  ContiRadarConfig200& UpdateRadarAllConfig(struct can_frame* candata) {
    uint32_t RadarCfg_ID =
        (0x200 | (static_cast<uint32_t>(previous_sensor_ID_) << 4));

    candata->can_id = RadarCfg_ID;
    candata->can_dlc = 6;
    for (int i = 0; i != 6; ++i) {
      candata->data[i] = 0x00;
    }

    // If the validity bit is set to valid (0x1), the corresponding
    // parameter will be updated in the ARS, otherwise it is ignored.

    switch (RadarType) {
      case 0: {
        set_max_distance_valid_p(candata->data, true);
        set_max_distance_p(candata->data, Input_ContiRadar_Config_.MaxDistance);
      } break;
      case 1: {
        set_max_distance_valid_p(candata->data, true);
        set_max_distance_p(candata->data, Input_ContiRadar_Config_.MaxDistance);
      } break;
      case 2: {
        set_max_distance_valid_p(candata->data, false);
        set_max_distance_p(candata->data, Input_ContiRadar_Config_.MaxDistance);
      } break;
      default:
        break;
    }  // end switch

    set_sensor_id_valid_p(candata->data, true);
    set_sensor_id_p(candata->data, Input_ContiRadar_Config_.SensorID);

    set_radar_power_valid_p(candata->data, true);
    set_radar_power_p(candata->data, Input_ContiRadar_Config_.RadarPower);

    set_output_type_valid_p(candata->data, true);
    set_output_type_p(candata->data, Input_ContiRadar_Config_.OutputType);

    set_send_quality_valid_p(candata->data, true);
    set_send_quality_p(candata->data, Input_ContiRadar_Config_.SendQuality);

    set_send_ext_info_valid_p(candata->data, true);
    set_send_ext_info_p(candata->data, Input_ContiRadar_Config_.SendExtInfo);

    set_sort_index_valid_p(candata->data, true);
    set_sort_index_p(candata->data, Input_ContiRadar_Config_.SortIndex);

    set_ctrl_relay_valid_p(candata->data, true);
    set_ctrl_relay_p(candata->data, Input_ContiRadar_Config_.CtrlRelay);

    set_rcs_threshold_valid_p(candata->data, true);
    set_rcs_threshold_p(candata->data, Input_ContiRadar_Config_.RCS_Threshold);

    set_store_in_nvm_valid_p(candata->data, true);
    set_store_in_nvm_p(candata->data, 0x1);

    return *this;

  }  // UpdateRadarAllConfig

  auto GetPreviousContiRadarID() const noexcept { return previous_sensor_ID_; }
  auto GetPreviousContiRadarConfig() const noexcept {
    return Read_Previous_Radar_Config_;
  }
  auto GetContiRadarConfig() const noexcept {
    return Read_Current_Radar_Config_;
  }

 private:
  // Previous Radar Config
  uint8_t previous_sensor_ID_;  // input
  ContiRadarConfig Read_Previous_Radar_Config_;

  // User-input info
  ContiRadarConfig Input_ContiRadar_Config_;

  // Read out from radar After configuration
  ContiRadarConfig Read_Current_Radar_Config_;

  /*************************  0x200  ***************************/
  void set_max_distance_valid_p(uint8_t* data, const bool valid) {
    if (valid) {
      data[0] |= 0b00000001;
    } else {
      data[0] |= 0x00;
    }
  }  // set_max_distance_valid_p
  void set_sensor_id_valid_p(uint8_t* data, const bool valid) {
    if (valid) {
      data[0] |= 0b00000010;
    } else {
      data[0] |= 0x00;
    }
  }  // set_sensor_id_valid_p
  void set_radar_power_valid_p(uint8_t* data, const bool valid) {
    if (valid) {
      data[0] |= 0b00000100;
    } else {
      data[0] |= 0x00;
    }
  }  // set_radar_power_valid_p
  void set_output_type_valid_p(uint8_t* data, const bool valid) {
    if (valid) {
      data[0] |= 0b00001000;
    } else {
      data[0] |= 0x00;
    }
  }  // set_output_type_valid_p
  void set_send_quality_valid_p(uint8_t* data, const bool valid) {
    if (valid) {
      data[0] |= 0b00010000;
    } else {
      data[0] |= 0x00;
    }
  }  // set_send_quality_valid_p
  void set_send_ext_info_valid_p(uint8_t* data, const bool valid) {
    if (valid) {
      data[0] |= 0b00100000;
    } else {
      data[0] |= 0x00;
    }
  }  // set_send_ext_info_valid_p
  void set_sort_index_valid_p(uint8_t* data, const bool valid) {
    if (valid) {
      data[0] |= 0b01000000;
    } else {
      data[0] |= 0x00;
    }
  }  // set_sort_index_valid_p
  void set_store_in_nvm_valid_p(uint8_t* data, const bool valid) {
    if (valid) {
      data[0] |= 0b10000000;
    } else {
      data[0] |= 0x00;
    }
  }  // set_store_in_nvm_valid_p
  void set_ctrl_relay_valid_p(uint8_t* data, const bool valid) {
    uint8_t value = 0x00;
    if (valid) {
      value = 0x01;
    }
    data[5] |= (value);
  }  // set_ctrl_relay_valid_p
  void set_rcs_threshold_valid_p(uint8_t* data, const bool valid) {
    uint8_t value = 0x00;
    if (valid) {
      value = 0x01;
    }
    data[6] |= (value);
  }  // set_rcs_threshold_valid_p
  void set_max_distance_p(uint8_t* data, const uint16_t value) {
    uint16_t _value = value / 2;
    data[1] = static_cast<uint8_t>(_value >> 2);
    data[2] = static_cast<uint8_t>(_value << 6) & 0b11000000;
  }  // set_max_distance_p
  void set_sensor_id_p(uint8_t* data, const uint8_t value) {
    data[4] |= (value & 0b111);
  }  // set_sensor_id_p
  void set_output_type_p(uint8_t* data, const uint8_t value) {
    data[4] |= ((value << 3) & 0b00011000);
  }  // set_output_type_p
  void set_radar_power_p(uint8_t* data, const uint8_t value) {
    data[4] |= ((value << 5) & 0b11100000);
  }  // set_radar_power_p
  void set_ctrl_relay_p(uint8_t* data, const uint8_t value) {
    data[5] |= ((value << 1) & 0b00000010);
  }  // set_ctrl_relay_p
  void set_send_ext_info_p(uint8_t* data, const uint8_t value) {
    data[5] |= ((value << 3) & 0b00001000);
  }  // set_send_ext_info_p
  void set_send_quality_p(uint8_t* data, const uint8_t value) {
    data[5] |= ((value << 2) & 0b00000100);
  }  // set_send_quality_p
  void set_sort_index_p(uint8_t* data, const uint8_t value) {
    data[5] |= ((value << 4) & 0b01110000);
  }  // set_sort_index_p
  void set_store_in_nvm_p(uint8_t* data, const uint8_t value) {
    data[5] |= ((value << 7) & 0b10000000);
  }  // set_store_in_nvm_p
  void set_rcs_threshold_p(uint8_t* data, const uint8_t rcs_threshold) {
    data[6] |= ((rcs_threshold << 1) & 0b1110);
  }  // set_rcs_threshold_p

  /*************************  0x201  ***************************/
  uint8_t parse_NVMread_status(const uint8_t* bytes) const {
    return ((bytes[0] & 0b1000000) >> 6);
  }  // parse_NVMread_status
  uint16_t parse_max_dist(const uint8_t* bytes) const {
    uint16_t x = (static_cast<uint16_t>(bytes[1]) << 2) |
                 static_cast<uint16_t>(bytes[2] >> 6);
    return x * 2;
  }  // parse_max_dist
  uint8_t parse_radar_power(const uint8_t* bytes) const {
    return ((bytes[3] & 0b11) << 1) | (bytes[4] >> 7);
  }  // parse_radar_power
  uint8_t parse_output_type(const uint8_t* bytes) const {
    return ((bytes[5] & 0b1100) >> 2);
  }  // parse_output_type
  uint8_t parse_rcs_threshold(const uint8_t* bytes) const {
    return ((bytes[7] & 0b11100) >> 2);
  }  // parse_rcs_threshold
  uint8_t parse_send_quality(const uint8_t* bytes) const {
    return ((bytes[5] & 0b10000) >> 4);
  }  // parse_send_quality
  uint8_t parse_send_ext_info(const uint8_t* bytes) const {
    return ((bytes[5] & 0b100000) >> 5);
  }  // parse_send_ext_info
  uint8_t parse_ctrl_relay(const uint8_t* bytes) const {
    return ((bytes[5] & 0b10) >> 1);
  }  // parse_ctrl_relay
  uint8_t parse_sensor_ID(const uint8_t* bytes) const {
    return (bytes[4] & 0b111);
  }  // parse_sensor_ID
  uint8_t parse_sort_index(const uint8_t* bytes) const {
    return ((bytes[4] & 0b01110000) >> 4);
  }  // parse_sort_index

  /*************************  0x700  ***************************/
  uint8_t parse_major_release(const uint8_t* bytes) const {
    return bytes[0];
  }  // parse_major_release
  uint8_t parse_minor_release(const uint8_t* bytes) const {
    return bytes[1];
  }  // parse_minor_release
  uint8_t parse_patch_level(const uint8_t* bytes) const {
    return bytes[2];
  }  // parse_patch_level
  uint8_t parse_extended_range(const uint8_t* bytes) const {
    return ((bytes[3] & 0b10) >> 1);
  }  // parse_extended_range
  uint8_t parse_country_code(const uint8_t* bytes) const {
    return (bytes[3] & 0b1);
  }  // parse_country_code
};   // end class ContiRadarConfig200

#endif /* _ContiRadarConfig_200_H_ */
