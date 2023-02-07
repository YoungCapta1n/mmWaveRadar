/*
****************************************************************************
* mmWaveRadar_test.cc:
* unit test for target tracking using mmWaveRadar
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
****************************************************************************
*/

#include <chrono>
#include <ctime>
#include <string>
#include <thread>

#include "../CANBus.h"
#include "../ContiRadarObject_600.h"

class timecounter {
  using PTIMER = std::chrono::steady_clock;

 public:
  timecounter() : pt_start(PTIMER::now()){};
  ~timecounter() {}

  // return the elapsed duration in milliseconds
  long int timeelapsed() {
    auto pt_now = PTIMER::now();
    long int milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(pt_now - pt_start)
            .count();
    pt_start = pt_now;
    return milliseconds;
  }  // timeelapsed

  // return the elapsed duration in microseconds
  long long micro_timeelapsed() {
    auto pt_now = PTIMER::now();
    long long microseconds =
        std::chrono::duration_cast<std::chrono::microseconds>(pt_now - pt_start)
            .count();
    pt_start = pt_now;
    return microseconds;
  }  // micro_timeelapsed

  // return the UTC time (ISO)
  // TODO: C++20 support utc_time
  std::string getUTCtime() {
    std::time_t result = std::time(nullptr);
    std::string _utc = std::asctime(std::localtime(&result));
    _utc.pop_back();
    return _utc;
  }  // getUTCtime

 private:
  PTIMER::time_point pt_start;

};  // end class timecounter

ContiRadarObject_600<25> ARS40X_Driver(0x0);

void SinglemmWaveRadarCan() {
  CANBus CAN_Receiver("can0", "500000");

  timecounter Radar_timer;

  long long int outerloop_elapsed_time = 0;
  while (1) {
    struct can_frame frame;

    if (CAN_Receiver.ReceiveOneFrame(&frame) > 0) {
      printf("0x%03X [%d] ", frame.can_id, frame.can_dlc);
      for (int j = 0; j < frame.can_dlc; j++) printf("%02X ", frame.data[j]);
      printf("\r\n");

      ARS40X_Driver.ParseRadarData(&frame);

    }  // end if

    outerloop_elapsed_time = Radar_timer.micro_timeelapsed();
    printf("elapsed_time(us): %lld\n", outerloop_elapsed_time);
  }
}  // SinglemmWaveRadarCan

void mmWaveRadarDriver() {
  timecounter Radar_timer;

  long long int outerloop_elapsed_time = 0;
  while (1) {
    auto Is_Canbus_Idle = ARS40X_Driver.Check_Is_Canbus_Idle_RegenerateObject();

    printf("Is_Canbus_Idle: %d\n", Is_Canbus_Idle);
    if (Is_Canbus_Idle) {
      auto t_RadarObjectInfoARS = ARS40X_Driver.GetRadarObjectInfoARS();

      printf("RadarObjectInfoARS\n");
      for (uint32_t index = 0; index != t_RadarObjectInfoARS.num_of_object;
           ++index) {
        printf("time: %lf, %f, %f, %f, %f, %02x\n",
               t_RadarObjectInfoARS.Obj_timestamp[index],
               t_RadarObjectInfoARS.Obj_DistLong[index],
               t_RadarObjectInfoARS.Obj_DistLat[index],
               t_RadarObjectInfoARS.Obj_VrelLong[index],
               t_RadarObjectInfoARS.Obj_VrelLat[index],
               t_RadarObjectInfoARS.Obj_ProbOfExist[index]);
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // outerloop_elapsed_time = Radar_timer.micro_timeelapsed();
    // printf("elapsed_time(us): %lld\n", outerloop_elapsed_time);
  }
}  // SinglemmWaveRadarTest

int main() {
  std::thread t1(SinglemmWaveRadarCan);
  std::thread t2(mmWaveRadarDriver);

  t1.join();
  t2.join();
}