# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/Coding/OpenSource/Radar/test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/Coding/OpenSource/Radar/test/build

# Include any dependencies generated for this target.
include CMakeFiles/mmWaveRadarConfig.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/mmWaveRadarConfig.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/mmWaveRadarConfig.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mmWaveRadarConfig.dir/flags.make

CMakeFiles/mmWaveRadarConfig.dir/mmWaveRadarConfig.cc.o: CMakeFiles/mmWaveRadarConfig.dir/flags.make
CMakeFiles/mmWaveRadarConfig.dir/mmWaveRadarConfig.cc.o: ../mmWaveRadarConfig.cc
CMakeFiles/mmWaveRadarConfig.dir/mmWaveRadarConfig.cc.o: CMakeFiles/mmWaveRadarConfig.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Coding/OpenSource/Radar/test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mmWaveRadarConfig.dir/mmWaveRadarConfig.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/mmWaveRadarConfig.dir/mmWaveRadarConfig.cc.o -MF CMakeFiles/mmWaveRadarConfig.dir/mmWaveRadarConfig.cc.o.d -o CMakeFiles/mmWaveRadarConfig.dir/mmWaveRadarConfig.cc.o -c /home/pi/Coding/OpenSource/Radar/test/mmWaveRadarConfig.cc

CMakeFiles/mmWaveRadarConfig.dir/mmWaveRadarConfig.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mmWaveRadarConfig.dir/mmWaveRadarConfig.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Coding/OpenSource/Radar/test/mmWaveRadarConfig.cc > CMakeFiles/mmWaveRadarConfig.dir/mmWaveRadarConfig.cc.i

CMakeFiles/mmWaveRadarConfig.dir/mmWaveRadarConfig.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mmWaveRadarConfig.dir/mmWaveRadarConfig.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Coding/OpenSource/Radar/test/mmWaveRadarConfig.cc -o CMakeFiles/mmWaveRadarConfig.dir/mmWaveRadarConfig.cc.s

# Object files for target mmWaveRadarConfig
mmWaveRadarConfig_OBJECTS = \
"CMakeFiles/mmWaveRadarConfig.dir/mmWaveRadarConfig.cc.o"

# External object files for target mmWaveRadarConfig
mmWaveRadarConfig_EXTERNAL_OBJECTS =

mmWaveRadarConfig: CMakeFiles/mmWaveRadarConfig.dir/mmWaveRadarConfig.cc.o
mmWaveRadarConfig: CMakeFiles/mmWaveRadarConfig.dir/build.make
mmWaveRadarConfig: /usr/local/lib/libcontrolcan.so
mmWaveRadarConfig: CMakeFiles/mmWaveRadarConfig.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/Coding/OpenSource/Radar/test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable mmWaveRadarConfig"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mmWaveRadarConfig.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mmWaveRadarConfig.dir/build: mmWaveRadarConfig
.PHONY : CMakeFiles/mmWaveRadarConfig.dir/build

CMakeFiles/mmWaveRadarConfig.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mmWaveRadarConfig.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mmWaveRadarConfig.dir/clean

CMakeFiles/mmWaveRadarConfig.dir/depend:
	cd /home/pi/Coding/OpenSource/Radar/test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/Coding/OpenSource/Radar/test /home/pi/Coding/OpenSource/Radar/test /home/pi/Coding/OpenSource/Radar/test/build /home/pi/Coding/OpenSource/Radar/test/build /home/pi/Coding/OpenSource/Radar/test/build/CMakeFiles/mmWaveRadarConfig.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mmWaveRadarConfig.dir/depend

