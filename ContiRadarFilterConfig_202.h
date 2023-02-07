/*
****************************************************************************
* ContiRadarFilterConfig_202.h:
* Receive Data from Continental mmWaveRadar (ARS40X), etc.
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#ifndef _ContiRadarFilterConfig_202_H_
#define _ContiRadarFilterConfig_202_H_

#include <linux/can.h>

#include "ContiRadarData.h"

constexpr uint8_t FILTER_NofObj = 0x0;
constexpr uint8_t FILTER_Distance = 0x1;
constexpr uint8_t FILTER_Azimuth = 0x2;
constexpr uint8_t FILTER_VrelOncome = 0x3;
constexpr uint8_t FILTER_VrelDepart = 0x4;
constexpr uint8_t FILTER_RCS = 0x5;
constexpr uint8_t FILTER_Lifetime = 0x6;
constexpr uint8_t FILTER_Size = 0x7;
constexpr uint8_t FILTER_ProbExists = 0x8;
constexpr uint8_t FILTER_Y = 0x9;
constexpr uint8_t FILTER_X = 0xA;
constexpr uint8_t FILTER_VYRightLeft = 0xB;
constexpr uint8_t FILTER_VXOncome = 0xC;
constexpr uint8_t FILTER_VYLeftRight = 0xD;
constexpr uint8_t FILTER_VXDepart = 0xE;

struct RadarFilterStatusARS {
  uint8_t Type;

  /******** 0x0 ********/
  bool NofObj_Active;
  uint16_t Min_NofObj;
  uint16_t Max_NofObj;

  /******** 0x1 ********/
  bool Distance_Active;
  float Min_Distance;
  float Max_Distance;

  /******** 0x2 ********/
  bool Azimuth_Active;
  float Min_Azimuth;
  float Max_Azimuth;

  /******** 0x3 ********/
  bool VrelOncome_Active;
  float Min_VrelOncome;
  float Max_VrelOncome;

  /******** 0x4 ********/
  bool VrelDepart_Active;
  float Min_VrelDepart;
  float Max_VrelDepart;

  /******** 0x5 ********/
  bool RCS_Active;
  float Min_RCS;
  float Max_RCS;

  /******** 0x6 ********/
  bool Lifetime_Active;
  float Min_Lifetime;
  float Max_Lifetime;

  /******** 0x7 ********/
  bool Size_Active;
  float Min_Size;
  float Max_Size;

  /******** 0x8 ********/
  bool ProbExists_Active;
  uint16_t Min_ProbExists;
  uint16_t Max_ProbExists;

  /******** 0x9 ********/
  bool Y_Active;
  float Min_Y;
  float Max_Y;

  /******** 0xA ********/
  bool X_Active;
  float Min_X;
  float Max_X;

  /******** 0xB ********/
  bool VYRightLeft_Active;
  float Min_VYRightLeft;
  float Max_VYRightLeft;

  /******** 0xC ********/
  bool VXOncome_Active;
  float Min_VXOncome;
  float Max_VXOncome;

  /******** 0xD ********/
  bool VYLeftRight_Active;
  float Min_VYLeftRight;
  float Max_VYLeftRight;

  /******** 0xE ********/
  bool VXDepart_Active;
  float Min_VXDepart;
  float Max_VXDepart;
};  // RadarFilterStatusARS

class ContiRadarFilterConfig202 {
 public:
  ContiRadarFilterConfig202() : sensor_ID_(0x0) {}
  ~ContiRadarFilterConfig202() {}

  ContiRadarFilterConfig202& SetupSensorID(const uint8_t sensorID) {
    if (sensorID <= 0x07) {
      sensor_ID_ = sensorID;
    } else
      sensor_ID_ = 0x0;
    return *this;
  }  // SetupSensorID

  ContiRadarFilterConfig202& UpdateFilterConfig(
      const RadarFilterStatusARS& RadarFilter_config) {
    input_RFilter_ = RadarFilter_config;
    return *this;
  }  // SetupSensorID

  ContiRadarFilterConfig202& SetupFilterConfig(
      struct can_frame* candata, const uint8_t filterstate_index) {
    uint32_t FilterCfg_ID = (0x202 | (static_cast<uint32_t>(sensor_ID_) << 4));

    candata->can_id = FilterCfg_ID;
    candata->can_dlc = 5;
    for (int i = 0; i != 5; ++i) {
      candata->data[i] = 0x00;
    }

    set_filter_state_valid(candata->data, 0x1);
    set_filter_state_type(candata->data, input_RFilter_.Type);

    switch (filterstate_index) {
      case FILTER_NofObj: {
        set_filter_state_index(candata->data, FILTER_NofObj);

        if (input_RFilter_.NofObj_Active)
          set_filter_state_active(candata->data, 0x1);
        else
          set_filter_state_active(candata->data, 0x0);
        set_min_numofobj(candata->data, input_RFilter_.Min_NofObj);
        set_max_numofobj(candata->data, input_RFilter_.Max_NofObj);
      } break;
      case FILTER_Distance: {
        set_filter_state_index(candata->data, FILTER_Distance);

        if (input_RFilter_.Distance_Active)
          set_filter_state_active(candata->data, 0x1);
        else
          set_filter_state_active(candata->data, 0x0);
        set_min_distance(candata->data, input_RFilter_.Min_Distance);
        set_max_distance(candata->data, input_RFilter_.Max_Distance);
      } break;
      case FILTER_Azimuth: {
        set_filter_state_index(candata->data, FILTER_Azimuth);

        if (input_RFilter_.Azimuth_Active)
          set_filter_state_active(candata->data, 0x1);
        else
          set_filter_state_active(candata->data, 0x0);
        set_min_azimuth(candata->data, input_RFilter_.Min_Azimuth);
        set_max_azimuth(candata->data, input_RFilter_.Max_Azimuth);
      } break;
      case FILTER_VrelOncome: {
        set_filter_state_index(candata->data, FILTER_VrelOncome);

        if (input_RFilter_.VrelOncome_Active)
          set_filter_state_active(candata->data, 0x1);
        else
          set_filter_state_active(candata->data, 0x0);
        set_min_vrel_oncome(candata->data, input_RFilter_.Min_VrelOncome);
        set_max_vrel_oncome(candata->data, input_RFilter_.Max_VrelOncome);
      } break;
      case FILTER_VrelDepart: {
        set_filter_state_index(candata->data, FILTER_VrelDepart);

        if (input_RFilter_.VrelDepart_Active)
          set_filter_state_active(candata->data, 0x1);
        else
          set_filter_state_active(candata->data, 0x0);
        set_min_vrel_depart(candata->data, input_RFilter_.Min_VrelDepart);
        set_max_vrel_depart(candata->data, input_RFilter_.Max_VrelDepart);
      } break;
      case FILTER_RCS: {
        set_filter_state_index(candata->data, FILTER_RCS);

        if (input_RFilter_.RCS_Active)
          set_filter_state_active(candata->data, 0x1);
        else
          set_filter_state_active(candata->data, 0x0);
        set_min_rcs(candata->data, input_RFilter_.Min_RCS);
        set_max_rcs(candata->data, input_RFilter_.Max_RCS);
      } break;
      case FILTER_Lifetime: {
        set_filter_state_index(candata->data, FILTER_Lifetime);

        if (input_RFilter_.Lifetime_Active)
          set_filter_state_active(candata->data, 0x1);
        else
          set_filter_state_active(candata->data, 0x0);
        set_min_lifetime(candata->data, input_RFilter_.Min_Lifetime);
        set_max_lifetime(candata->data, input_RFilter_.Max_Lifetime);
      } break;
      case FILTER_Size: {
        set_filter_state_index(candata->data, FILTER_Size);

        if (input_RFilter_.Size_Active)
          set_filter_state_active(candata->data, 0x1);
        else
          set_filter_state_active(candata->data, 0x0);
        set_min_size(candata->data, input_RFilter_.Min_Size);
        set_max_size(candata->data, input_RFilter_.Max_Size);
      } break;
      case FILTER_ProbExists: {
        set_filter_state_index(candata->data, FILTER_ProbExists);

        if (input_RFilter_.ProbExists_Active)
          set_filter_state_active(candata->data, 0x1);
        else
          set_filter_state_active(candata->data, 0x0);
        set_min_probexists(candata->data, input_RFilter_.Min_ProbExists);
        set_max_probexists(candata->data, input_RFilter_.Max_ProbExists);
      } break;
      case FILTER_Y: {
        set_filter_state_index(candata->data, FILTER_Y);

        if (input_RFilter_.Y_Active)
          set_filter_state_active(candata->data, 0x1);
        else
          set_filter_state_active(candata->data, 0x0);
        set_min_y(candata->data, input_RFilter_.Min_Y);
        set_max_y(candata->data, input_RFilter_.Max_Y);
      } break;
      case FILTER_X: {
        set_filter_state_index(candata->data, FILTER_X);

        if (input_RFilter_.X_Active)
          set_filter_state_active(candata->data, 0x1);
        else
          set_filter_state_active(candata->data, 0x0);
        set_min_x(candata->data, input_RFilter_.Min_X);
        set_max_x(candata->data, input_RFilter_.Max_X);
      } break;
      case FILTER_VYRightLeft: {
        set_filter_state_index(candata->data, FILTER_VYRightLeft);

        if (input_RFilter_.VYRightLeft_Active)
          set_filter_state_active(candata->data, 0x1);
        else
          set_filter_state_active(candata->data, 0x0);
        set_min_vy_rightleft(candata->data, input_RFilter_.Min_VYRightLeft);
        set_max_vy_rightleft(candata->data, input_RFilter_.Max_VYRightLeft);
      } break;
      case FILTER_VXOncome: {
        set_filter_state_index(candata->data, FILTER_VXOncome);

        if (input_RFilter_.VXOncome_Active)
          set_filter_state_active(candata->data, 0x1);
        else
          set_filter_state_active(candata->data, 0x0);
        set_min_vx_oncome(candata->data, input_RFilter_.Min_VXOncome);
        set_max_vx_oncome(candata->data, input_RFilter_.Max_VXOncome);
      } break;
      case FILTER_VYLeftRight: {
        set_filter_state_index(candata->data, FILTER_VYLeftRight);

        if (input_RFilter_.VYLeftRight_Active)
          set_filter_state_active(candata->data, 0x1);
        else
          set_filter_state_active(candata->data, 0x0);
        set_min_vy_leftright(candata->data, input_RFilter_.Min_VYLeftRight);
        set_max_vy_leftright(candata->data, input_RFilter_.Max_VYLeftRight);
      } break;
      case FILTER_VXDepart: {
        set_filter_state_index(candata->data, FILTER_VXDepart);

        if (input_RFilter_.VXDepart_Active)
          set_filter_state_active(candata->data, 0x1);
        else
          set_filter_state_active(candata->data, 0x0);
        set_min_vx_depart(candata->data, input_RFilter_.Min_VXDepart);
        set_max_vx_depart(candata->data, input_RFilter_.Max_VXDepart);
      } break;

      default:
        break;
    }  // end switch

    return *this;
  }  // SetupFilterConfig

  bool HandleFilterInfo(const struct can_frame* candata) {
    uint32_t FilterState_Cfg_ID =
        (0x204 | (static_cast<uint32_t>(sensor_ID_) << 4));

    if (candata->can_id == FilterState_Cfg_ID) {
      radar_filter_.Type = parse_filter_state_type(candata->data);
      uint8_t filter_index = parse_filter_state_index(candata->data);
      switch (filter_index) {
        case FILTER_NofObj: {
          radar_filter_.NofObj_Active =
              (parse_filter_state_active(candata->data) == 0x1);
          radar_filter_.Min_NofObj = parse_min_numofobj(candata->data);
          radar_filter_.Max_NofObj = parse_max_numofobj(candata->data);
        } break;
        case FILTER_Distance: {
          radar_filter_.Distance_Active =
              (parse_filter_state_active(candata->data) == 0x1);
          radar_filter_.Min_Distance = parse_min_distance(candata->data);
          radar_filter_.Max_Distance = parse_max_distance(candata->data);
        } break;
        case FILTER_Azimuth: {
          radar_filter_.Azimuth_Active =
              (parse_filter_state_active(candata->data) == 0x1);
          radar_filter_.Min_Azimuth = parse_min_azimuth(candata->data);
          radar_filter_.Max_Azimuth = parse_max_azimuth(candata->data);
        } break;
        case FILTER_VrelOncome: {
          radar_filter_.VrelOncome_Active =
              (parse_filter_state_active(candata->data) == 0x1);
          radar_filter_.Min_VrelOncome = parse_min_vrel_oncome(candata->data);
          radar_filter_.Max_VrelOncome = parse_max_vrel_oncome(candata->data);
        } break;
        case FILTER_VrelDepart: {
          radar_filter_.VrelDepart_Active =
              (parse_filter_state_active(candata->data) == 0x1);
          radar_filter_.Min_VrelDepart = parse_min_vrel_depart(candata->data);
          radar_filter_.Max_VrelDepart = parse_max_vrel_depart(candata->data);
        } break;
        case FILTER_RCS: {
          radar_filter_.RCS_Active =
              (parse_filter_state_active(candata->data) == 0x1);
          radar_filter_.Min_RCS = parse_min_rcs(candata->data);
          radar_filter_.Max_RCS = parse_max_rcs(candata->data);
        } break;
        case FILTER_Lifetime: {
          radar_filter_.Lifetime_Active =
              (parse_filter_state_active(candata->data) == 0x1);
          radar_filter_.Min_Lifetime = parse_min_lifetime(candata->data);
          radar_filter_.Max_Lifetime = parse_max_lifetime(candata->data);
        } break;
        case FILTER_Size: {
          radar_filter_.Size_Active =
              (parse_filter_state_active(candata->data) == 0x1);
          radar_filter_.Min_Size = parse_min_size(candata->data);
          radar_filter_.Max_Size = parse_max_size(candata->data);
        } break;
        case FILTER_ProbExists: {
          radar_filter_.ProbExists_Active =
              (parse_filter_state_active(candata->data) == 0x1);
          radar_filter_.Min_ProbExists = parse_min_probexists(candata->data);
          radar_filter_.Max_ProbExists = parse_max_probexists(candata->data);
        } break;
        case FILTER_Y: {
          radar_filter_.Y_Active =
              (parse_filter_state_active(candata->data) == 0x1);
          radar_filter_.Min_Y = parse_min_y(candata->data);
          radar_filter_.Max_Y = parse_max_y(candata->data);
        } break;
        case FILTER_X: {
          radar_filter_.X_Active =
              (parse_filter_state_active(candata->data) == 0x1);
          radar_filter_.Min_X = parse_min_x(candata->data);
          radar_filter_.Max_X = parse_max_x(candata->data);
        } break;
        case FILTER_VYRightLeft: {
          radar_filter_.VYRightLeft_Active =
              (parse_filter_state_active(candata->data) == 0x1);
          radar_filter_.Min_VYRightLeft = parse_min_vy_rightleft(candata->data);
          radar_filter_.Max_VYRightLeft = parse_max_vy_rightleft(candata->data);
        } break;
        case FILTER_VXOncome: {
          radar_filter_.VXOncome_Active =
              (parse_filter_state_active(candata->data) == 0x1);
          radar_filter_.Min_VXOncome = parse_min_vx_oncome(candata->data);
          radar_filter_.Max_VXOncome = parse_max_vx_oncome(candata->data);
        } break;
        case FILTER_VYLeftRight: {
          radar_filter_.VYLeftRight_Active =
              (parse_filter_state_active(candata->data) == 0x1);
          radar_filter_.Min_VYLeftRight = parse_min_vy_leftright(candata->data);
          radar_filter_.Max_VYLeftRight = parse_max_vy_leftright(candata->data);
        } break;
        case FILTER_VXDepart: {
          radar_filter_.VXDepart_Active =
              (parse_filter_state_active(candata->data) == 0x1);
          radar_filter_.Min_VXDepart = parse_min_vx_depart(candata->data);
          radar_filter_.Max_VXDepart = parse_max_vx_depart(candata->data);
        } break;

        default:
          break;
      }  // end switch

      return true;
    }  // end if

    return false;

  }  // HandleFilterInfo

  bool CheckRadarAllFilterConfig() {
    if (input_RFilter_.Type != radar_filter_.Type) return false;

    if (input_RFilter_.NofObj_Active != radar_filter_.NofObj_Active)
      return false;
    if (radar_filter_.NofObj_Active == 0x1) {
      if (input_RFilter_.Min_NofObj != radar_filter_.Min_NofObj) return false;
      if (input_RFilter_.Max_NofObj != radar_filter_.Max_NofObj) return false;
    }

    if (input_RFilter_.Distance_Active != radar_filter_.Distance_Active)
      return false;
    if (radar_filter_.Distance_Active == 0x1) {
      if (input_RFilter_.Min_Distance != radar_filter_.Min_Distance)
        return false;
      if (input_RFilter_.Max_Distance != radar_filter_.Max_Distance)
        return false;
    }

    if (input_RFilter_.Azimuth_Active != radar_filter_.Azimuth_Active)
      return false;
    if (radar_filter_.Azimuth_Active == 0x1) {
      if (input_RFilter_.Min_Azimuth != radar_filter_.Min_Azimuth) return false;
      if (input_RFilter_.Max_Azimuth != radar_filter_.Max_Azimuth) return false;
    }

    if (input_RFilter_.VrelOncome_Active != radar_filter_.VrelOncome_Active)
      return false;
    if (radar_filter_.VrelOncome_Active == 0x1) {
      if (input_RFilter_.Min_VrelOncome != radar_filter_.Min_VrelOncome)
        return false;
      if (input_RFilter_.Max_VrelOncome != radar_filter_.Max_VrelOncome)
        return false;
    }

    if (input_RFilter_.VrelDepart_Active != radar_filter_.VrelDepart_Active)
      return false;
    if (radar_filter_.VrelDepart_Active == 0x1) {
      if (input_RFilter_.Min_VrelDepart != radar_filter_.Min_VrelDepart)
        return false;
      if (input_RFilter_.Max_VrelDepart != radar_filter_.Max_VrelDepart)
        return false;
    }

    if (input_RFilter_.RCS_Active != radar_filter_.RCS_Active) return false;
    if (radar_filter_.RCS_Active == 0x1) {
      if (input_RFilter_.Min_RCS != radar_filter_.Min_RCS) return false;
      if (input_RFilter_.Max_RCS != radar_filter_.Max_RCS) return false;
    }

    if (radar_filter_.Type == 0x1) {
      if (input_RFilter_.Lifetime_Active != radar_filter_.Lifetime_Active)
        return false;
      if (radar_filter_.Lifetime_Active == 0x1) {
        if (input_RFilter_.Min_Lifetime != radar_filter_.Min_Lifetime)
          return false;
        if (input_RFilter_.Max_Lifetime != radar_filter_.Max_Lifetime)
          return false;
      }

      if (input_RFilter_.Size_Active != radar_filter_.Size_Active) return false;
      if (radar_filter_.Size_Active == 0x1) {
        if (input_RFilter_.Min_Size != radar_filter_.Min_Size) return false;
        if (input_RFilter_.Max_Size != radar_filter_.Max_Size) return false;
      }

      if (input_RFilter_.ProbExists_Active != radar_filter_.ProbExists_Active)
        return false;
      if (radar_filter_.ProbExists_Active == 0x1) {
        if (input_RFilter_.Min_ProbExists != radar_filter_.Min_ProbExists)
          return false;
        if (input_RFilter_.Max_ProbExists != radar_filter_.Max_ProbExists)
          return false;
      }

      if (input_RFilter_.Y_Active != radar_filter_.Y_Active) return false;
      if (radar_filter_.Y_Active == 0x1) {
        if (input_RFilter_.Min_Y != radar_filter_.Min_Y) return false;
        if (input_RFilter_.Max_Y != radar_filter_.Max_Y) return false;
      }

      if (input_RFilter_.X_Active != radar_filter_.X_Active) return false;
      if (radar_filter_.X_Active == 0x1) {
        if (input_RFilter_.Min_X != radar_filter_.Min_X) return false;
        if (input_RFilter_.Max_X != radar_filter_.Max_X) return false;
      }

      if (input_RFilter_.VYRightLeft_Active != radar_filter_.VYRightLeft_Active)
        return false;
      if (radar_filter_.VYRightLeft_Active == 0x1) {
        if (input_RFilter_.Min_VYRightLeft != radar_filter_.Min_VYRightLeft)
          return false;
        if (input_RFilter_.Max_VYRightLeft != radar_filter_.Max_VYRightLeft)
          return false;
      }

      if (input_RFilter_.VXOncome_Active != radar_filter_.VXOncome_Active)
        return false;
      if (radar_filter_.VXOncome_Active == 0x1) {
        if (input_RFilter_.Min_VXOncome != radar_filter_.Min_VXOncome)
          return false;
        if (input_RFilter_.Max_VXOncome != radar_filter_.Max_VXOncome)
          return false;
      }

      if (input_RFilter_.VYLeftRight_Active != radar_filter_.VYLeftRight_Active)
        return false;
      if (radar_filter_.VYLeftRight_Active == 0x1) {
        if (input_RFilter_.Min_VYLeftRight != radar_filter_.Min_VYLeftRight)
          return false;
        if (input_RFilter_.Max_VYLeftRight != radar_filter_.Max_VYLeftRight)
          return false;
      }

      if (input_RFilter_.VXDepart_Active != radar_filter_.VXDepart_Active)
        return false;
      if (radar_filter_.VXDepart_Active == 0x1) {
        if (input_RFilter_.Min_VXDepart != radar_filter_.Min_VXDepart)
          return false;
        if (input_RFilter_.Max_VXDepart != radar_filter_.Max_VXDepart)
          return false;
      }
    }

    return true;
  }  // CheckRadarAllFilterConfig

  auto GetRadarFilterStatusARS() const noexcept { return radar_filter_; }

 private:
  uint8_t sensor_ID_;
  RadarFilterStatusARS input_RFilter_;
  RadarFilterStatusARS radar_filter_;

  /**************************** 0x203 ******************************/
  uint8_t parse_num_cluster_filter_cfg(const uint8_t* bytes) const {
    return bytes[0] >> 3;
  }  // parse_num_cluster_filter_cfg
  uint8_t parse_num_object_filter_cfg(const uint8_t* bytes) const {
    return bytes[1] >> 3;
  }  // parse_num_object_filter_cfg

  /**************************** 0x204 ******************************/
  uint8_t parse_filter_state_active(const uint8_t* bytes) const {
    return (bytes[0] & 0b100) >> 2;
  }  // parse_filter_state_active
  uint8_t parse_filter_state_index(const uint8_t* bytes) const {
    return (bytes[0] & 0b1111000) >> 3;
  }  // parse_filter_state_index
  uint8_t parse_filter_state_type(const uint8_t* bytes) const {
    return bytes[0] >> 7;
  }  // parse_filter_state_type
  uint16_t parse_min_numofobj(const uint8_t* bytes) const {
    return (static_cast<uint16_t>(bytes[1] & 0b1111) << 8) |
           static_cast<uint16_t>(bytes[2]);
  }  // parse_min_numofobj
  float parse_min_distance(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[1] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[2]);
    return 0.1 * static_cast<float>(_value);
  }  // parse_min_distance
  float parse_min_azimuth(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[1] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[2]);
    return 0.025 * static_cast<float>(_value) - 50.0;
  }  // parse_min_azimuth
  float parse_min_vrel_oncome(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[1] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[2]);
    return 0.0315 * static_cast<float>(_value);
  }  // parse_min_vrel_oncome
  float parse_min_vrel_depart(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[1] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[2]);
    return 0.0315 * static_cast<float>(_value);
  }  // parse_min_vrel_depart
  float parse_min_rcs(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[1] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[2]);
    return 0.025 * static_cast<float>(_value) - 50.0;
  }  // parse_min_rcs
  float parse_min_lifetime(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[1] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[2]);
    return 0.1 * static_cast<float>(_value);
  }  // parse_min_lifetime
  float parse_min_size(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[1] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[2]);
    return 0.025 * static_cast<float>(_value);
  }  // parse_min_size
  uint16_t parse_min_probexists(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[1] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[2]);
    return _value;
  }  // parse_min_probexists
  float parse_min_y(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[1] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[2]);
    return 0.2 * static_cast<float>(_value) - 409.5;
  }  // parse_min_y
  float parse_min_x(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[1] & 0b11111) << 8) |
                      static_cast<uint16_t>(bytes[2]);
    return 0.2 * static_cast<float>(_value) - 500.0;
  }  // parse_min_x
  float parse_min_vy_leftright(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[1] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[2]);
    return 0.0315 * static_cast<float>(_value);
  }  // parse_min_vy_leftright
  float parse_min_vx_oncome(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[1] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[2]);
    return 0.0315 * static_cast<float>(_value);
  }  // parse_min_vx_oncome
  float parse_min_vy_rightleft(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[1] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[2]);
    return 0.0315 * static_cast<float>(_value);
  }  // parse_min_vy_rightleft
  float parse_min_vx_depart(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[1] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[2]);
    return 0.0315 * static_cast<float>(_value);
  }  // parse_min_vx_depart
  uint16_t parse_max_numofobj(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[3] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[4]);
    return _value;
  }  // parse_max_numofobj
  float parse_max_distance(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[3] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[4]);
    return 0.1 * static_cast<float>(_value);
  }  // parse_max_distance
  float parse_max_azimuth(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[3] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[4]);
    return 0.025 * static_cast<float>(_value) - 50.0;
  }  // parse_max_azimuth
  float parse_max_vrel_oncome(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[3] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[4]);
    return 0.0315 * static_cast<float>(_value);
  }  // parse_max_vrel_oncome
  float parse_max_vrel_depart(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[3] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[4]);
    return 0.0315 * static_cast<float>(_value);
  }  // parse_max_vrel_depart
  float parse_max_rcs(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[3] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[4]);
    return 0.025 * static_cast<float>(_value) - 50.0;
  }  // parse_max_rcs
  float parse_max_lifetime(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[3] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[4]);
    return 0.1 * static_cast<float>(_value);
  }  // parse_max_lifetime
  float parse_max_size(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[3] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[4]);
    return 0.025 * static_cast<float>(_value);
  }  // parse_max_size
  uint16_t parse_max_probexists(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[3] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[4]);
    return _value;
  }  // parse_max_probexists
  float parse_max_y(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[3] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[4]);
    return 0.2 * static_cast<float>(_value) - 409.5;
  }  // parse_max_y
  float parse_max_x(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[3] & 0b11111) << 8) |
                      static_cast<uint16_t>(bytes[4]);
    return 0.2 * static_cast<float>(_value) - 500.0;
  }  // parse_max_x
  float parse_max_vy_leftright(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[3] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[4]);
    return 0.0315 * static_cast<float>(_value);
  }  // parse_max_vy_leftright
  float parse_max_vx_oncome(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[3] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[4]);
    return 0.0315 * static_cast<float>(_value);
  }  // parse_max_vx_oncome
  float parse_max_vy_rightleft(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[3] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[4]);
    return 0.0315 * static_cast<float>(_value);
  }  // parse_max_vy_rightleft
  float parse_max_vx_depart(const uint8_t* bytes) const {
    uint16_t _value = (static_cast<uint16_t>(bytes[3] & 0b1111) << 8) |
                      static_cast<uint16_t>(bytes[4]);
    return 0.0315 * static_cast<float>(_value);
  }  // parse_max_vx_depart

  /**************************** 0x202 ******************************/
  void set_filter_state_valid(uint8_t* data, const uint8_t value) {
    data[0] |= (value << 1);
  }  // set_filter_state_valid
  void set_filter_state_active(uint8_t* data, const uint8_t value) {
    data[0] |= (value << 2);
  }  // set_filter_state_active
  void set_filter_state_index(uint8_t* data, const uint8_t value) {
    data[0] |= (value << 3);
  }  // set_filter_state_index
  void set_filter_state_type(uint8_t* data, const uint8_t value) {
    data[0] |= (value << 7);
  }  // set_filter_state_type
  void set_min_numofobj(uint8_t* data, const uint16_t value) {
    data[1] = static_cast<uint8_t>(value >> 8);
    data[2] = static_cast<uint8_t>(value);
  }  // set_min_numofobj
  void set_min_distance(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>(value * 10.0);
    data[1] = static_cast<uint8_t>(_value >> 8);
    data[2] = static_cast<uint8_t>(_value);
  }  // set_min_distance
  void set_min_azimuth(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>((value + 50.0) * 40.0);
    data[1] = static_cast<uint8_t>(_value >> 8);
    data[2] = static_cast<uint8_t>(_value);
  }  // set_min_azimuth
  void set_min_vrel_oncome(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>(value / 0.0315);
    data[1] = static_cast<uint8_t>(_value >> 8);
    data[2] = static_cast<uint8_t>(_value);
  }  // set_min_vrel_oncome
  void set_min_vrel_depart(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>(value / 0.0315);
    data[1] = static_cast<uint8_t>(_value >> 8);
    data[2] = static_cast<uint8_t>(_value);
  }  // set_min_vrel_depart
  void set_min_rcs(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>((value + 50.0) * 40.0);
    data[1] = static_cast<uint8_t>(_value >> 8);
    data[2] = static_cast<uint8_t>(_value);
  }  // set_min_rcs
  void set_min_lifetime(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>(value * 10.0);
    data[1] = static_cast<uint8_t>(_value >> 8);
    data[2] = static_cast<uint8_t>(_value);
  }  // set_min_lifetime
  void set_min_size(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>(value * 40.0);
    data[1] = static_cast<uint8_t>(_value >> 8);
    data[2] = static_cast<uint8_t>(_value);
  }  // set_min_size
  void set_min_probexists(uint8_t* data, const uint16_t value) {
    data[1] = static_cast<uint8_t>(value >> 8);
    data[2] = static_cast<uint8_t>(value);
  }  // set_min_probexists
  void set_min_y(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>((value + 409.5) * 5.0);
    data[1] = static_cast<uint8_t>(_value >> 8);
    data[2] = static_cast<uint8_t>(_value);
  }  // set_min_y
  void set_min_x(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>((value + 500.0) * 5.0);
    data[1] = static_cast<uint8_t>(_value >> 8);
    data[2] = static_cast<uint8_t>(_value);
  }  // set_min_x
  void set_min_vy_leftright(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>(value / 0.0315);
    data[1] = static_cast<uint8_t>(_value >> 8);
    data[2] = static_cast<uint8_t>(_value);
  }  // set_min_vy_leftright
  void set_min_vx_oncome(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>(value / 0.0315);
    data[1] = static_cast<uint8_t>(_value >> 8);
    data[2] = static_cast<uint8_t>(_value);
  }  // set_min_vx_oncome
  void set_min_vy_rightleft(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>(value / 0.0315);
    data[1] = static_cast<uint8_t>(_value >> 8);
    data[2] = static_cast<uint8_t>(_value);
  }  // set_min_vy_rightleft
  void set_min_vx_depart(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>(value / 0.0315);
    data[1] = static_cast<uint8_t>(_value >> 8);
    data[2] = static_cast<uint8_t>(_value);
  }  // set_min_vx_depart
  void set_max_numofobj(uint8_t* data, const uint16_t value) {
    data[3] = static_cast<uint8_t>(value >> 8);
    data[4] = static_cast<uint8_t>(value);
  }  // set_max_numofobj
  void set_max_distance(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>(value * 10.0);
    data[3] = static_cast<uint8_t>(_value >> 8);
    data[4] = static_cast<uint8_t>(_value);
  }  // set_max_distance
  void set_max_azimuth(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>((value + 50.0) * 40.0);
    data[3] = static_cast<uint8_t>(_value >> 8);
    data[4] = static_cast<uint8_t>(_value);
  }  // set_max_azimuth
  void set_max_vrel_oncome(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>(value / 0.0315);
    data[3] = static_cast<uint8_t>(_value >> 8);
    data[4] = static_cast<uint8_t>(_value);
  }  // set_max_vrel_oncome
  void set_max_vrel_depart(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>(value / 0.0315);
    data[3] = static_cast<uint8_t>(_value >> 8);
    data[4] = static_cast<uint8_t>(_value);
  }  // set_max_vrel_depart
  void set_max_rcs(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>((value + 50.0) * 40.0);
    data[3] = static_cast<uint8_t>(_value >> 8);
    data[4] = static_cast<uint8_t>(_value);
  }  // set_max_rcs
  void set_max_lifetime(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>(value * 10.0);
    data[3] = static_cast<uint8_t>(_value >> 8);
    data[4] = static_cast<uint8_t>(_value);
  }  // set_max_lifetime
  void set_max_size(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>(value * 40.0);
    data[3] = static_cast<uint8_t>(_value >> 8);
    data[4] = static_cast<uint8_t>(_value);
  }  // set_max_size
  void set_max_probexists(uint8_t* data, const uint16_t value) {
    data[3] = static_cast<uint8_t>(value >> 8);
    data[4] = static_cast<uint8_t>(value);
  }  // set_max_probexists
  void set_max_y(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>((value + 409.5) * 5.0);
    data[3] = static_cast<uint8_t>(_value >> 8);
    data[4] = static_cast<uint8_t>(_value);
  }  // set_max_y
  void set_max_x(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>((value + 500.0) * 5.0);
    data[3] = static_cast<uint8_t>(_value >> 8);
    data[4] = static_cast<uint8_t>(_value);
  }  // set_max_x
  void set_max_vy_leftright(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>(value / 0.0315);
    data[3] = static_cast<uint8_t>(_value >> 8);
    data[4] = static_cast<uint8_t>(_value);
  }  // set_max_vy_leftright
  void set_max_vx_oncome(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>(value / 0.0315);
    data[3] = static_cast<uint8_t>(_value >> 8);
    data[4] = static_cast<uint8_t>(_value);
  }  // set_max_vx_oncome
  void set_max_vy_rightleft(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>(value / 0.0315);
    data[3] = static_cast<uint8_t>(_value >> 8);
    data[4] = static_cast<uint8_t>(_value);
  }  // set_max_vy_rightleft
  void set_max_vx_depart(uint8_t* data, const float value) {
    uint16_t _value = static_cast<uint16_t>(value / 0.0315);
    data[3] = static_cast<uint8_t>(_value >> 8);
    data[4] = static_cast<uint8_t>(_value);
  }  // set_max_vx_depart

};  // end class ContiRadarFilterConfig202

#endif /* _ContiRadarFilterConfig_200_H_ */