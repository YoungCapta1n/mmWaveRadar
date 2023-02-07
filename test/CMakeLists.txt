# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.10)

# 项目信息
project (testradar)
set(CMAKE_CXX_STANDARD 17)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES arm)
message("-- Build on " ${CMAKE_SYSTEM_PROCESSOR})
option(BUILD_ON_ARM "build on arm or not" ON)
else()
message("-- Build on " ${CMAKE_SYSTEM_PROCESSOR})
option(BUILD_ON_ARM "build on arm or not" OFF)
endif()


if(BUILD_ON_ARM)
add_compile_definitions(_RASPBERRYPI_)
set(CMAKE_BUILD_TYPE "Release") # "Debug" or "Release" mode
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wextra -Wno-psabi -g -ggdb -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wno-psabi -mtune=cortex-a72")
set(CMAKE_INCLUDE_CURRENT_DIR ON)

else()

set(CMAKE_BUILD_TYPE "Debug") # "Debug" or "Release" mode
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wextra -Wno-psabi -g -ggdb -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -march=native -mavx")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_library(ZLGCAN_LIBRARY controlcan HINTS ${LIBRARY_DIRECTORY})

endif(BUILD_ON_ARM)

# 添加 include 子目录
set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../../"
	)

# thread库
find_package(Threads MODULE REQUIRED)

add_executable (mmWaveRadar_test mmWaveRadar_test.cc ${SOURCE_FILES})
target_include_directories(mmWaveRadar_test PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(mmWaveRadar_test ${CMAKE_THREAD_LIBS_INIT})

add_executable (mmWaveRadarConfig mmWaveRadarConfig.cc)
target_include_directories(mmWaveRadarConfig PUBLIC ${HEADER_DIRECTORY})
if(BUILD_ON_ARM)
else()
target_link_libraries(mmWaveRadarConfig PUBLIC ${ZLGCAN_LIBRARY})
endif(BUILD_ON_ARM)
