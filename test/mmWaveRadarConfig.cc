/*
****************************************************************************
* mmWaveRadarConfig.cc:
* setup the mmWaveRadar
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#ifdef _RASPBERRYPI_
#include "../CANBus.h"
#else
#include "../ZLGCANBus.h"
#endif

#include <iostream>
#include <thread>

#include "../ContiRadarConfig_200.h"
#include "../ContiRadarFilterConfig_202.h"

// 0: ARS408-21
// 1: ARS404-21
// 2: SRR308-21
constexpr int RadarType = 0;

constexpr uint8_t New_SensorID = 0x0;
constexpr uint16_t New_MaxDistance = 200;
constexpr uint8_t New_OutputType = 0x1;
constexpr bool New_SendQuality = false;
constexpr bool New_SendExtInfo = true;
constexpr uint8_t New_SortIndex = 0x0;

constexpr RadarFilterStatusARS RadarFilter_Config{
    0x1,    // Type
    true,   // NofObj_Active
    0,      // Min_NofObj
    40,     // Max_NofObj
    true,   // Distance_Active
    0,      // Min_Distance
    50,     // Max_Distance
    true,   // Azimuth_Active
    -40,    // Min_Azimuth
    40,     // Max_Azimuth
    false,  // VrelOncome_Active
    0,      // Min_VrelOncome
    1,      // Max_VrelOncome
    false,  // VrelDepart_Active
    0,      // Min_VrelDepart
    1,      // Max_VrelDepart
    false,  // RCS_Active
    0,      // Min_RCS
    1,      // Max_RCS
    false,  // Lifetime_Active
    0,      // Min_Lifetime
    1,      // Max_Lifetime
    false,  // Size_Active
    0,      // Min_Size
    1,      // Max_Size
    true,   // ProbExists_Active
    0x4,    // Min_ProbExists
    0x7,    // Max_ProbExists
    false,  // Y_Active
    0,      // Min_Y
    1,      // Max_Y
    false,  // X_Active
    0,      // Min_X
    1,      // Max_X
    false,  // VYRightLeft_Active
    0,      // Min_VYRightLeft
    1,      // Max_VYRightLeft
    false,  // VXOncome_Active
    0,      // Min_VXOncome
    1,      // Max_VXOncome
    false,  // VYLeftRight_Active
    0,      // Min_VYLeftRight
    1,      // Max_VYLeftRight
    false,  // VXDepart_Active
    0,      // Min_VXDepart
    1       // Max_VXDepart
};

int main() {
#ifdef _RASPBERRYPI_
  CANBus CAN_Receiver("can0", "500000");
#else
  ZLGCANBus CAN_Receiver(500000);
#endif

  ContiRadarConfig200<RadarType> ContiRadar_Config;
  std::cout << "Radar Type: " << ContiRadar_Config.MapRadarType2String()
            << std::endl;

  // parse previous Radar Config
  struct can_frame frame;
  while (1) {
    if (CAN_Receiver.ReceiveOneFrame(&frame) > 0) {
      /*
      printf("0x%03X [%d] ", frame.can_id, frame.can_dlc);
      for (int j = 0; j < frame.can_dlc; j++) printf("%02X ", frame.data[j]);
      printf("\r\n");
      */
      if (ContiRadar_Config.ParsePreviousRadarConfig(&frame)) {
        printf("success in reading pervious radar configuration!\n");

        uint8_t SensorID = ContiRadar_Config.GetPreviousContiRadarID();
        auto Read_Previous_Radar_Config =
            ContiRadar_Config.GetPreviousContiRadarConfig();
        printf(
            "******************* Previous Radar Config ******************\n");
        printf("SensorID:%02x\n", SensorID);
        printf("MaxDistance:%d\n", Read_Previous_Radar_Config.MaxDistance);
        printf("RadarPower:%02x\n", Read_Previous_Radar_Config.RadarPower);
        printf("OutputType:%02x\n", Read_Previous_Radar_Config.OutputType);
        printf("SendQuality:%02x\n", Read_Previous_Radar_Config.SendQuality);
        printf("SendExtInfo:%02x\n", Read_Previous_Radar_Config.SendExtInfo);
        printf("SortIndex:%02x\n", Read_Previous_Radar_Config.SortIndex);
        printf("CtrlRelay:%02x\n", Read_Previous_Radar_Config.CtrlRelay);
        printf("RCS_Threshold:%02x\n",
               Read_Previous_Radar_Config.RCS_Threshold);

        break;
      }
    }  // end if
  }

  // update the user-input config
  ContiRadar_Config
      .SetupInputContiRadarConfig(New_SensorID,     // new_sensor_ID
                                  New_MaxDistance,  // max_distance
                                  New_OutputType,   // output_type
                                  New_SendQuality,  // IsOutputObjectQuality
                                  New_SendExtInfo,  // IsOutputExtInfo
                                  New_SortIndex     // SortIndexType
                                  )
      .UpdateRadarAllConfig(&frame);
  CAN_Receiver.SendOneFrame(&frame);

  // Check if radar is configured successfully
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  while (1) {
    if (CAN_Receiver.ReceiveOneFrame(&frame) > 0) {
      int results = ContiRadar_Config.CheckCurrentConfigStatus(&frame);
      if (results == 1) {
        printf("mmWaveRadar is Configured Successfully!\n");
        auto Read_current_Radar_Config =
            ContiRadar_Config.GetContiRadarConfig();
        printf("**************** Current Radar Config ******************\n");
        printf("SensorID:%02x\n", Read_current_Radar_Config.SensorID);
        printf("MaxDistance:%d\n", Read_current_Radar_Config.MaxDistance);

        switch (Read_current_Radar_Config.RadarPower) {
          case 0x0:
            printf("RadarPower: Standard\n");
            break;
          case 0x1:
            printf("RadarPower: -3dB Tx gain\n");
            break;
          case 0x2:
            printf("RadarPower: -6dB Tx gain\n");
            break;
          case 0x3:
            printf("RadarPower: -9dB Tx gain\n");
            break;
          default:
            break;
        }

        switch (Read_current_Radar_Config.OutputType) {
          case 0x0:
            printf("OutputType: none\n");
            break;
          case 0x1:
            printf("OutputType: send objects\n");
            break;
          case 0x2:
            printf("OutputType: send clusters\n");
            break;
          default:
            break;
        }

        if (Read_current_Radar_Config.SendQuality == 0x1)
          printf("SendQuality: active\n");
        else
          printf("SendQuality: inactive\n");

        if (Read_current_Radar_Config.SendExtInfo == 0x1)
          printf("SendExtInfo: active\n");
        else
          printf("SendExtInfo: inactive\n");

        switch (Read_current_Radar_Config.SortIndex) {
          case 0x0:
            printf("SortIndex: no sorting\n");
            break;
          case 0x1:
            printf("SortIndex: sorted by range\n");
            break;
          case 0x2:
            printf("SortIndex: sorted by RCS\n");
            break;
          default:
            break;
        }
        if (Read_current_Radar_Config.CtrlRelay == 0x1)
          printf("CtrlRelay: active\n");
        else
          printf("CtrlRelay: inactive\n");
        if (Read_current_Radar_Config.RCS_Threshold == 0x1)
          printf("RCS_Threshold: high sensitivity\n");
        else
          printf("RCS_Threshold: standard\n");

        /**********************  Filter Config ***********************/
        ContiRadarFilterConfig202 ContiRadar_FilterConfig;
        ContiRadar_FilterConfig.SetupSensorID(New_SensorID)
            .UpdateFilterConfig(RadarFilter_Config);

        uint8_t end_index = RadarFilter_Config.Type == 0x1 ? 0xF : 0x6;
        for (uint8_t index = 0x0; index != end_index; ++index) {
          // update the user-input filter config
          ContiRadar_FilterConfig.SetupFilterConfig(&frame, index);
          CAN_Receiver.SendOneFrame(&frame);

          std::this_thread::sleep_for(std::chrono::milliseconds(1));

          while (1) {
            if (CAN_Receiver.ReceiveOneFrame(&frame) > 0) {
              if (ContiRadar_FilterConfig.HandleFilterInfo(&frame)) {
                break;
              }
            }
          }  // end while
        }    // end for loop

        if (ContiRadar_FilterConfig.CheckRadarAllFilterConfig()) {
          auto output_radar_filter =
              ContiRadar_FilterConfig.GetRadarFilterStatusARS();
          printf("Radar Filter is Configured Successfully!\n");
          printf("************ Current Radar Filter Config ************\n");
          if (output_radar_filter.Type == 0x1)
            printf("Type: Object Filter\n");
          else
            printf("Type: Cluster Filter\n");

          if (output_radar_filter.NofObj_Active)
            printf("NofObj: Active\n");
          else
            printf("NofObj: Inactive\n");
          printf("NofObj min/max: %d, %d\n", output_radar_filter.Min_NofObj,
                 output_radar_filter.Max_NofObj);

          if (output_radar_filter.Distance_Active)
            printf("Distance: Active\n");
          else
            printf("Distance: Inactive\n");
          printf("Distance min/max: %f, %f\n", output_radar_filter.Min_Distance,
                 output_radar_filter.Max_Distance);

          if (output_radar_filter.Azimuth_Active)
            printf("Azimuth: Active\n");
          else
            printf("Azimuth: Inactive\n");
          printf("Azimuth min/max: %f, %f\n", output_radar_filter.Min_Azimuth,
                 output_radar_filter.Max_Azimuth);

          if (output_radar_filter.VrelOncome_Active)
            printf("VrelOncome: Active\n");
          else
            printf("VrelOncome: Inactive\n");
          printf("VrelOncome min/max: %f, %f\n",
                 output_radar_filter.Min_VrelOncome,
                 output_radar_filter.Max_VrelOncome);

          if (output_radar_filter.VrelDepart_Active)
            printf("VrelDepart: Active\n");
          else
            printf("VrelDepart: Inactive\n");
          printf("VrelDepart min/max: %f, %f\n",
                 output_radar_filter.Min_VrelDepart,
                 output_radar_filter.Max_VrelDepart);

          if (output_radar_filter.RCS_Active)
            printf("RCS: Active\n");
          else
            printf("RCS: Inactive\n");
          printf("RCS min/max: %f, %f\n", output_radar_filter.Min_RCS,
                 output_radar_filter.Max_RCS);

          if (output_radar_filter.Lifetime_Active)
            printf("Lifetime: Active\n");
          else
            printf("Lifetime: Inactive\n");
          printf("Lifetime min/max: %f, %f\n", output_radar_filter.Min_Lifetime,
                 output_radar_filter.Max_Lifetime);

          if (output_radar_filter.Size_Active)
            printf("Size: Active\n");
          else
            printf("Size: Inactive\n");
          printf("Size min/max: %f, %f\n", output_radar_filter.Min_Size,
                 output_radar_filter.Max_Size);

          if (output_radar_filter.ProbExists_Active)
            printf("ProbExists: Active\n");
          else
            printf("ProbExists: Inactive\n");
          printf("ProbExists min/max: %d, %d\n",
                 output_radar_filter.Min_ProbExists,
                 output_radar_filter.Max_ProbExists);

          if (output_radar_filter.Y_Active)
            printf("Y: Active\n");
          else
            printf("Y: Inactive\n");
          printf("Y min/max: %f, %f\n", output_radar_filter.Min_Y,
                 output_radar_filter.Max_Y);

          if (output_radar_filter.X_Active)
            printf("X: Active\n");
          else
            printf("X: Inactive\n");
          printf("X min/max: %f, %f\n", output_radar_filter.Min_X,
                 output_radar_filter.Max_X);

          if (output_radar_filter.VYRightLeft_Active)
            printf("VYRightLeft: Active\n");
          else
            printf("VYRightLeft: Inactive\n");
          printf("VYRightLeft min/max: %f, %f\n",
                 output_radar_filter.Min_VYRightLeft,
                 output_radar_filter.Max_VYRightLeft);

          if (output_radar_filter.VXOncome_Active)
            printf("VXOncome: Active\n");
          else
            printf("VXOncome: Inactive\n");
          printf("VXOncome min/max: %f, %f\n", output_radar_filter.Min_VXOncome,
                 output_radar_filter.Max_VXOncome);

          if (output_radar_filter.VYLeftRight_Active)
            printf("VYLeftRight: Active\n");
          else
            printf("VYLeftRight: Inactive\n");
          printf("VYLeftRight min/max: %f, %f\n",
                 output_radar_filter.Min_VYLeftRight,
                 output_radar_filter.Max_VYLeftRight);

          if (output_radar_filter.VXDepart_Active)
            printf("VXDepart: Active\n");
          else
            printf("VXDepart: Inactive\n");
          printf("VXDepart min/max: %f, %f\n", output_radar_filter.Min_VXDepart,
                 output_radar_filter.Max_VXDepart);
        }  // end if
        else
          printf("error in Filter configuration\n");

        break;
      } else
        continue;

    }  // end if
  }
}
