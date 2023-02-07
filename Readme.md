## [代码]**C++ driver for Continental ARS408/404 Radar**

This is a C++ driver for Continental ARS 408-21 (ARS404-21/SRR308) automotive radar. The code is tested on Raspberry Pi 4B and X86 Linux.

大陆 ARS 408-21 (ARS404-21/SRR308) 毫米波雷达的C++驱动

## Install and Build

1. Make sure the CANBus hardware and its driver is successfully installed.

   - on Raspberry Pi 4B, MCP2515 can be used (https://www.waveshare.net/wiki/2-CH_CAN_HAT)
   - on x86 Linux, usb-CAN is used(https://www.zhcxgd.com/cn/ZLXZ.html), copy the `libcontrolcan.so` to `/usr/local/lib`

2. Make sure the mmWaveRadar is powered, and CANBus channels (CAN-H, CAN-L) are connected to each other.

3.  clone this repo.

   ```shell
   git clone https://github.com/AtsushiSakai/PythonRobotics.git
   ```

4. Create `build` directory and change directory

   ```shell
   cd mmWaveRadar
   mkdir build
   cd build
   ```

4. Compile examples

   ```shell
   cmake ..
   make
   ```



## Bug reports and support

Please report any issues via the [Github issue tracker](https://github.com/osqp/osqp/issues). All types of issues are welcome including bug reports, documentation typos, feature requests and so on.




