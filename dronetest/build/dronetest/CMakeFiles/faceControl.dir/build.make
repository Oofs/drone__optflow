# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /opt/dronetest/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /opt/dronetest/build

# Include any dependencies generated for this target.
include dronetest/CMakeFiles/faceControl.dir/depend.make

# Include the progress variables for this target.
include dronetest/CMakeFiles/faceControl.dir/progress.make

# Include the compile flags for this target's objects.
include dronetest/CMakeFiles/faceControl.dir/flags.make

dronetest/CMakeFiles/faceControl.dir/src/face/main_faceControl.cpp.o: dronetest/CMakeFiles/faceControl.dir/flags.make
dronetest/CMakeFiles/faceControl.dir/src/face/main_faceControl.cpp.o: /opt/dronetest/src/dronetest/src/face/main_faceControl.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /opt/dronetest/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object dronetest/CMakeFiles/faceControl.dir/src/face/main_faceControl.cpp.o"
	cd /opt/dronetest/build/dronetest && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/faceControl.dir/src/face/main_faceControl.cpp.o -c /opt/dronetest/src/dronetest/src/face/main_faceControl.cpp

dronetest/CMakeFiles/faceControl.dir/src/face/main_faceControl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/faceControl.dir/src/face/main_faceControl.cpp.i"
	cd /opt/dronetest/build/dronetest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /opt/dronetest/src/dronetest/src/face/main_faceControl.cpp > CMakeFiles/faceControl.dir/src/face/main_faceControl.cpp.i

dronetest/CMakeFiles/faceControl.dir/src/face/main_faceControl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/faceControl.dir/src/face/main_faceControl.cpp.s"
	cd /opt/dronetest/build/dronetest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /opt/dronetest/src/dronetest/src/face/main_faceControl.cpp -o CMakeFiles/faceControl.dir/src/face/main_faceControl.cpp.s

dronetest/CMakeFiles/faceControl.dir/src/face/main_faceControl.cpp.o.requires:
.PHONY : dronetest/CMakeFiles/faceControl.dir/src/face/main_faceControl.cpp.o.requires

dronetest/CMakeFiles/faceControl.dir/src/face/main_faceControl.cpp.o.provides: dronetest/CMakeFiles/faceControl.dir/src/face/main_faceControl.cpp.o.requires
	$(MAKE) -f dronetest/CMakeFiles/faceControl.dir/build.make dronetest/CMakeFiles/faceControl.dir/src/face/main_faceControl.cpp.o.provides.build
.PHONY : dronetest/CMakeFiles/faceControl.dir/src/face/main_faceControl.cpp.o.provides

dronetest/CMakeFiles/faceControl.dir/src/face/main_faceControl.cpp.o.provides.build: dronetest/CMakeFiles/faceControl.dir/src/face/main_faceControl.cpp.o

dronetest/CMakeFiles/faceControl.dir/src/face/faceControl.cpp.o: dronetest/CMakeFiles/faceControl.dir/flags.make
dronetest/CMakeFiles/faceControl.dir/src/face/faceControl.cpp.o: /opt/dronetest/src/dronetest/src/face/faceControl.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /opt/dronetest/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object dronetest/CMakeFiles/faceControl.dir/src/face/faceControl.cpp.o"
	cd /opt/dronetest/build/dronetest && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/faceControl.dir/src/face/faceControl.cpp.o -c /opt/dronetest/src/dronetest/src/face/faceControl.cpp

dronetest/CMakeFiles/faceControl.dir/src/face/faceControl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/faceControl.dir/src/face/faceControl.cpp.i"
	cd /opt/dronetest/build/dronetest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /opt/dronetest/src/dronetest/src/face/faceControl.cpp > CMakeFiles/faceControl.dir/src/face/faceControl.cpp.i

dronetest/CMakeFiles/faceControl.dir/src/face/faceControl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/faceControl.dir/src/face/faceControl.cpp.s"
	cd /opt/dronetest/build/dronetest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /opt/dronetest/src/dronetest/src/face/faceControl.cpp -o CMakeFiles/faceControl.dir/src/face/faceControl.cpp.s

dronetest/CMakeFiles/faceControl.dir/src/face/faceControl.cpp.o.requires:
.PHONY : dronetest/CMakeFiles/faceControl.dir/src/face/faceControl.cpp.o.requires

dronetest/CMakeFiles/faceControl.dir/src/face/faceControl.cpp.o.provides: dronetest/CMakeFiles/faceControl.dir/src/face/faceControl.cpp.o.requires
	$(MAKE) -f dronetest/CMakeFiles/faceControl.dir/build.make dronetest/CMakeFiles/faceControl.dir/src/face/faceControl.cpp.o.provides.build
.PHONY : dronetest/CMakeFiles/faceControl.dir/src/face/faceControl.cpp.o.provides

dronetest/CMakeFiles/faceControl.dir/src/face/faceControl.cpp.o.provides.build: dronetest/CMakeFiles/faceControl.dir/src/face/faceControl.cpp.o

# Object files for target faceControl
faceControl_OBJECTS = \
"CMakeFiles/faceControl.dir/src/face/main_faceControl.cpp.o" \
"CMakeFiles/faceControl.dir/src/face/faceControl.cpp.o"

# External object files for target faceControl
faceControl_EXTERNAL_OBJECTS =

/opt/dronetest/devel/lib/dronetest/faceControl: dronetest/CMakeFiles/faceControl.dir/src/face/main_faceControl.cpp.o
/opt/dronetest/devel/lib/dronetest/faceControl: dronetest/CMakeFiles/faceControl.dir/src/face/faceControl.cpp.o
/opt/dronetest/devel/lib/dronetest/faceControl: dronetest/CMakeFiles/faceControl.dir/build.make
/opt/dronetest/devel/lib/dronetest/faceControl: /opt/ros/indigo/lib/libcv_bridge.so
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_videostab.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_video.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_superres.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_stitching.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_photo.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_ocl.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_objdetect.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_ml.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_legacy.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_imgproc.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_highgui.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_gpu.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_flann.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_features2d.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_core.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_contrib.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_calib3d.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /opt/ros/indigo/lib/libimage_transport.so
/opt/dronetest/devel/lib/dronetest/faceControl: /opt/ros/indigo/lib/libmessage_filters.so
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libtinyxml.so
/opt/dronetest/devel/lib/dronetest/faceControl: /opt/ros/indigo/lib/libclass_loader.so
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/libPocoFoundation.so
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libdl.so
/opt/dronetest/devel/lib/dronetest/faceControl: /opt/ros/indigo/lib/libroscpp.so
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libboost_signals.so
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libboost_filesystem.so
/opt/dronetest/devel/lib/dronetest/faceControl: /opt/ros/indigo/lib/librosconsole.so
/opt/dronetest/devel/lib/dronetest/faceControl: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/opt/dronetest/devel/lib/dronetest/faceControl: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/liblog4cxx.so
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libboost_regex.so
/opt/dronetest/devel/lib/dronetest/faceControl: /opt/ros/indigo/lib/libxmlrpcpp.so
/opt/dronetest/devel/lib/dronetest/faceControl: /opt/ros/indigo/lib/libroslib.so
/opt/dronetest/devel/lib/dronetest/faceControl: /opt/ros/indigo/lib/libroscpp_serialization.so
/opt/dronetest/devel/lib/dronetest/faceControl: /opt/ros/indigo/lib/librostime.so
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libboost_date_time.so
/opt/dronetest/devel/lib/dronetest/faceControl: /opt/ros/indigo/lib/libcpp_common.so
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libboost_system.so
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libboost_thread.so
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libpthread.so
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libconsole_bridge.so
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_videostab.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_video.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_superres.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_stitching.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_photo.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_ocl.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_objdetect.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_ml.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_legacy.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_imgproc.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_highgui.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_gpu.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_flann.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_features2d.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_core.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_contrib.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_calib3d.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_photo.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_legacy.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_video.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_objdetect.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_ml.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_calib3d.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_features2d.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_highgui.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_imgproc.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_flann.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: /usr/lib/i386-linux-gnu/libopencv_core.so.2.4.8
/opt/dronetest/devel/lib/dronetest/faceControl: dronetest/CMakeFiles/faceControl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /opt/dronetest/devel/lib/dronetest/faceControl"
	cd /opt/dronetest/build/dronetest && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/faceControl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
dronetest/CMakeFiles/faceControl.dir/build: /opt/dronetest/devel/lib/dronetest/faceControl
.PHONY : dronetest/CMakeFiles/faceControl.dir/build

dronetest/CMakeFiles/faceControl.dir/requires: dronetest/CMakeFiles/faceControl.dir/src/face/main_faceControl.cpp.o.requires
dronetest/CMakeFiles/faceControl.dir/requires: dronetest/CMakeFiles/faceControl.dir/src/face/faceControl.cpp.o.requires
.PHONY : dronetest/CMakeFiles/faceControl.dir/requires

dronetest/CMakeFiles/faceControl.dir/clean:
	cd /opt/dronetest/build/dronetest && $(CMAKE_COMMAND) -P CMakeFiles/faceControl.dir/cmake_clean.cmake
.PHONY : dronetest/CMakeFiles/faceControl.dir/clean

dronetest/CMakeFiles/faceControl.dir/depend:
	cd /opt/dronetest/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /opt/dronetest/src /opt/dronetest/src/dronetest /opt/dronetest/build /opt/dronetest/build/dronetest /opt/dronetest/build/dronetest/CMakeFiles/faceControl.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : dronetest/CMakeFiles/faceControl.dir/depend

