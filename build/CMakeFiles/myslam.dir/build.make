# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /home/giri/myslam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/giri/myslam/build

# Include any dependencies generated for this target.
include CMakeFiles/myslam.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/myslam.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/myslam.dir/flags.make

CMakeFiles/myslam.dir/src/camera.cpp.o: CMakeFiles/myslam.dir/flags.make
CMakeFiles/myslam.dir/src/camera.cpp.o: ../src/camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/giri/myslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/myslam.dir/src/camera.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/src/camera.cpp.o -c /home/giri/myslam/src/camera.cpp

CMakeFiles/myslam.dir/src/camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/src/camera.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/giri/myslam/src/camera.cpp > CMakeFiles/myslam.dir/src/camera.cpp.i

CMakeFiles/myslam.dir/src/camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/src/camera.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/giri/myslam/src/camera.cpp -o CMakeFiles/myslam.dir/src/camera.cpp.s

CMakeFiles/myslam.dir/src/camera.cpp.o.requires:

.PHONY : CMakeFiles/myslam.dir/src/camera.cpp.o.requires

CMakeFiles/myslam.dir/src/camera.cpp.o.provides: CMakeFiles/myslam.dir/src/camera.cpp.o.requires
	$(MAKE) -f CMakeFiles/myslam.dir/build.make CMakeFiles/myslam.dir/src/camera.cpp.o.provides.build
.PHONY : CMakeFiles/myslam.dir/src/camera.cpp.o.provides

CMakeFiles/myslam.dir/src/camera.cpp.o.provides.build: CMakeFiles/myslam.dir/src/camera.cpp.o


CMakeFiles/myslam.dir/src/frame.cpp.o: CMakeFiles/myslam.dir/flags.make
CMakeFiles/myslam.dir/src/frame.cpp.o: ../src/frame.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/giri/myslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/myslam.dir/src/frame.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/src/frame.cpp.o -c /home/giri/myslam/src/frame.cpp

CMakeFiles/myslam.dir/src/frame.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/src/frame.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/giri/myslam/src/frame.cpp > CMakeFiles/myslam.dir/src/frame.cpp.i

CMakeFiles/myslam.dir/src/frame.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/src/frame.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/giri/myslam/src/frame.cpp -o CMakeFiles/myslam.dir/src/frame.cpp.s

CMakeFiles/myslam.dir/src/frame.cpp.o.requires:

.PHONY : CMakeFiles/myslam.dir/src/frame.cpp.o.requires

CMakeFiles/myslam.dir/src/frame.cpp.o.provides: CMakeFiles/myslam.dir/src/frame.cpp.o.requires
	$(MAKE) -f CMakeFiles/myslam.dir/build.make CMakeFiles/myslam.dir/src/frame.cpp.o.provides.build
.PHONY : CMakeFiles/myslam.dir/src/frame.cpp.o.provides

CMakeFiles/myslam.dir/src/frame.cpp.o.provides.build: CMakeFiles/myslam.dir/src/frame.cpp.o


CMakeFiles/myslam.dir/src/feature.cpp.o: CMakeFiles/myslam.dir/flags.make
CMakeFiles/myslam.dir/src/feature.cpp.o: ../src/feature.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/giri/myslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/myslam.dir/src/feature.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/src/feature.cpp.o -c /home/giri/myslam/src/feature.cpp

CMakeFiles/myslam.dir/src/feature.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/src/feature.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/giri/myslam/src/feature.cpp > CMakeFiles/myslam.dir/src/feature.cpp.i

CMakeFiles/myslam.dir/src/feature.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/src/feature.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/giri/myslam/src/feature.cpp -o CMakeFiles/myslam.dir/src/feature.cpp.s

CMakeFiles/myslam.dir/src/feature.cpp.o.requires:

.PHONY : CMakeFiles/myslam.dir/src/feature.cpp.o.requires

CMakeFiles/myslam.dir/src/feature.cpp.o.provides: CMakeFiles/myslam.dir/src/feature.cpp.o.requires
	$(MAKE) -f CMakeFiles/myslam.dir/build.make CMakeFiles/myslam.dir/src/feature.cpp.o.provides.build
.PHONY : CMakeFiles/myslam.dir/src/feature.cpp.o.provides

CMakeFiles/myslam.dir/src/feature.cpp.o.provides.build: CMakeFiles/myslam.dir/src/feature.cpp.o


CMakeFiles/myslam.dir/src/mappoint.cpp.o: CMakeFiles/myslam.dir/flags.make
CMakeFiles/myslam.dir/src/mappoint.cpp.o: ../src/mappoint.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/giri/myslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/myslam.dir/src/mappoint.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/src/mappoint.cpp.o -c /home/giri/myslam/src/mappoint.cpp

CMakeFiles/myslam.dir/src/mappoint.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/src/mappoint.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/giri/myslam/src/mappoint.cpp > CMakeFiles/myslam.dir/src/mappoint.cpp.i

CMakeFiles/myslam.dir/src/mappoint.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/src/mappoint.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/giri/myslam/src/mappoint.cpp -o CMakeFiles/myslam.dir/src/mappoint.cpp.s

CMakeFiles/myslam.dir/src/mappoint.cpp.o.requires:

.PHONY : CMakeFiles/myslam.dir/src/mappoint.cpp.o.requires

CMakeFiles/myslam.dir/src/mappoint.cpp.o.provides: CMakeFiles/myslam.dir/src/mappoint.cpp.o.requires
	$(MAKE) -f CMakeFiles/myslam.dir/build.make CMakeFiles/myslam.dir/src/mappoint.cpp.o.provides.build
.PHONY : CMakeFiles/myslam.dir/src/mappoint.cpp.o.provides

CMakeFiles/myslam.dir/src/mappoint.cpp.o.provides.build: CMakeFiles/myslam.dir/src/mappoint.cpp.o


CMakeFiles/myslam.dir/src/map.cpp.o: CMakeFiles/myslam.dir/flags.make
CMakeFiles/myslam.dir/src/map.cpp.o: ../src/map.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/giri/myslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/myslam.dir/src/map.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/src/map.cpp.o -c /home/giri/myslam/src/map.cpp

CMakeFiles/myslam.dir/src/map.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/src/map.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/giri/myslam/src/map.cpp > CMakeFiles/myslam.dir/src/map.cpp.i

CMakeFiles/myslam.dir/src/map.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/src/map.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/giri/myslam/src/map.cpp -o CMakeFiles/myslam.dir/src/map.cpp.s

CMakeFiles/myslam.dir/src/map.cpp.o.requires:

.PHONY : CMakeFiles/myslam.dir/src/map.cpp.o.requires

CMakeFiles/myslam.dir/src/map.cpp.o.provides: CMakeFiles/myslam.dir/src/map.cpp.o.requires
	$(MAKE) -f CMakeFiles/myslam.dir/build.make CMakeFiles/myslam.dir/src/map.cpp.o.provides.build
.PHONY : CMakeFiles/myslam.dir/src/map.cpp.o.provides

CMakeFiles/myslam.dir/src/map.cpp.o.provides.build: CMakeFiles/myslam.dir/src/map.cpp.o


CMakeFiles/myslam.dir/src/frontend.cpp.o: CMakeFiles/myslam.dir/flags.make
CMakeFiles/myslam.dir/src/frontend.cpp.o: ../src/frontend.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/giri/myslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/myslam.dir/src/frontend.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/src/frontend.cpp.o -c /home/giri/myslam/src/frontend.cpp

CMakeFiles/myslam.dir/src/frontend.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/src/frontend.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/giri/myslam/src/frontend.cpp > CMakeFiles/myslam.dir/src/frontend.cpp.i

CMakeFiles/myslam.dir/src/frontend.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/src/frontend.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/giri/myslam/src/frontend.cpp -o CMakeFiles/myslam.dir/src/frontend.cpp.s

CMakeFiles/myslam.dir/src/frontend.cpp.o.requires:

.PHONY : CMakeFiles/myslam.dir/src/frontend.cpp.o.requires

CMakeFiles/myslam.dir/src/frontend.cpp.o.provides: CMakeFiles/myslam.dir/src/frontend.cpp.o.requires
	$(MAKE) -f CMakeFiles/myslam.dir/build.make CMakeFiles/myslam.dir/src/frontend.cpp.o.provides.build
.PHONY : CMakeFiles/myslam.dir/src/frontend.cpp.o.provides

CMakeFiles/myslam.dir/src/frontend.cpp.o.provides.build: CMakeFiles/myslam.dir/src/frontend.cpp.o


CMakeFiles/myslam.dir/src/viewer.cpp.o: CMakeFiles/myslam.dir/flags.make
CMakeFiles/myslam.dir/src/viewer.cpp.o: ../src/viewer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/giri/myslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/myslam.dir/src/viewer.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/src/viewer.cpp.o -c /home/giri/myslam/src/viewer.cpp

CMakeFiles/myslam.dir/src/viewer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/src/viewer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/giri/myslam/src/viewer.cpp > CMakeFiles/myslam.dir/src/viewer.cpp.i

CMakeFiles/myslam.dir/src/viewer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/src/viewer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/giri/myslam/src/viewer.cpp -o CMakeFiles/myslam.dir/src/viewer.cpp.s

CMakeFiles/myslam.dir/src/viewer.cpp.o.requires:

.PHONY : CMakeFiles/myslam.dir/src/viewer.cpp.o.requires

CMakeFiles/myslam.dir/src/viewer.cpp.o.provides: CMakeFiles/myslam.dir/src/viewer.cpp.o.requires
	$(MAKE) -f CMakeFiles/myslam.dir/build.make CMakeFiles/myslam.dir/src/viewer.cpp.o.provides.build
.PHONY : CMakeFiles/myslam.dir/src/viewer.cpp.o.provides

CMakeFiles/myslam.dir/src/viewer.cpp.o.provides.build: CMakeFiles/myslam.dir/src/viewer.cpp.o


# Object files for target myslam
myslam_OBJECTS = \
"CMakeFiles/myslam.dir/src/camera.cpp.o" \
"CMakeFiles/myslam.dir/src/frame.cpp.o" \
"CMakeFiles/myslam.dir/src/feature.cpp.o" \
"CMakeFiles/myslam.dir/src/mappoint.cpp.o" \
"CMakeFiles/myslam.dir/src/map.cpp.o" \
"CMakeFiles/myslam.dir/src/frontend.cpp.o" \
"CMakeFiles/myslam.dir/src/viewer.cpp.o"

# External object files for target myslam
myslam_EXTERNAL_OBJECTS =

../lib/libmyslam.so: CMakeFiles/myslam.dir/src/camera.cpp.o
../lib/libmyslam.so: CMakeFiles/myslam.dir/src/frame.cpp.o
../lib/libmyslam.so: CMakeFiles/myslam.dir/src/feature.cpp.o
../lib/libmyslam.so: CMakeFiles/myslam.dir/src/mappoint.cpp.o
../lib/libmyslam.so: CMakeFiles/myslam.dir/src/map.cpp.o
../lib/libmyslam.so: CMakeFiles/myslam.dir/src/frontend.cpp.o
../lib/libmyslam.so: CMakeFiles/myslam.dir/src/viewer.cpp.o
../lib/libmyslam.so: CMakeFiles/myslam.dir/build.make
../lib/libmyslam.so: /usr/local/lib/libopencv_dnn.so.3.4.3
../lib/libmyslam.so: /usr/local/lib/libopencv_ml.so.3.4.3
../lib/libmyslam.so: /usr/local/lib/libopencv_objdetect.so.3.4.3
../lib/libmyslam.so: /usr/local/lib/libopencv_shape.so.3.4.3
../lib/libmyslam.so: /usr/local/lib/libopencv_stitching.so.3.4.3
../lib/libmyslam.so: /usr/local/lib/libopencv_superres.so.3.4.3
../lib/libmyslam.so: /usr/local/lib/libopencv_videostab.so.3.4.3
../lib/libmyslam.so: /usr/local/lib/libopencv_viz.so.3.4.3
../lib/libmyslam.so: /usr/local/lib/libpango_glgeometry.so
../lib/libmyslam.so: /usr/local/lib/libpango_plot.so
../lib/libmyslam.so: /usr/local/lib/libpango_python.so
../lib/libmyslam.so: /usr/local/lib/libpango_scene.so
../lib/libmyslam.so: /usr/local/lib/libpango_tools.so
../lib/libmyslam.so: /usr/local/lib/libpango_video.so
../lib/libmyslam.so: /usr/local/lib/libopencv_calib3d.so.3.4.3
../lib/libmyslam.so: /usr/local/lib/libopencv_features2d.so.3.4.3
../lib/libmyslam.so: /usr/local/lib/libopencv_flann.so.3.4.3
../lib/libmyslam.so: /usr/local/lib/libopencv_highgui.so.3.4.3
../lib/libmyslam.so: /usr/local/lib/libopencv_photo.so.3.4.3
../lib/libmyslam.so: /usr/local/lib/libopencv_video.so.3.4.3
../lib/libmyslam.so: /usr/local/lib/libopencv_videoio.so.3.4.3
../lib/libmyslam.so: /usr/local/lib/libopencv_imgcodecs.so.3.4.3
../lib/libmyslam.so: /usr/local/lib/libopencv_imgproc.so.3.4.3
../lib/libmyslam.so: /usr/local/lib/libopencv_core.so.3.4.3
../lib/libmyslam.so: /usr/local/lib/libpango_geometry.so
../lib/libmyslam.so: /usr/local/lib/libtinyobj.so
../lib/libmyslam.so: /usr/local/lib/libpango_display.so
../lib/libmyslam.so: /usr/local/lib/libpango_vars.so
../lib/libmyslam.so: /usr/local/lib/libpango_windowing.so
../lib/libmyslam.so: /usr/local/lib/libpango_opengl.so
../lib/libmyslam.so: /usr/lib/x86_64-linux-gnu/libGLEW.so
../lib/libmyslam.so: /usr/lib/x86_64-linux-gnu/libOpenGL.so
../lib/libmyslam.so: /usr/lib/x86_64-linux-gnu/libGLX.so
../lib/libmyslam.so: /usr/lib/x86_64-linux-gnu/libGLU.so
../lib/libmyslam.so: /usr/local/lib/libpango_image.so
../lib/libmyslam.so: /usr/local/lib/libpango_packetstream.so
../lib/libmyslam.so: /usr/local/lib/libpango_core.so
../lib/libmyslam.so: CMakeFiles/myslam.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/giri/myslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX shared library ../lib/libmyslam.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/myslam.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/myslam.dir/build: ../lib/libmyslam.so

.PHONY : CMakeFiles/myslam.dir/build

CMakeFiles/myslam.dir/requires: CMakeFiles/myslam.dir/src/camera.cpp.o.requires
CMakeFiles/myslam.dir/requires: CMakeFiles/myslam.dir/src/frame.cpp.o.requires
CMakeFiles/myslam.dir/requires: CMakeFiles/myslam.dir/src/feature.cpp.o.requires
CMakeFiles/myslam.dir/requires: CMakeFiles/myslam.dir/src/mappoint.cpp.o.requires
CMakeFiles/myslam.dir/requires: CMakeFiles/myslam.dir/src/map.cpp.o.requires
CMakeFiles/myslam.dir/requires: CMakeFiles/myslam.dir/src/frontend.cpp.o.requires
CMakeFiles/myslam.dir/requires: CMakeFiles/myslam.dir/src/viewer.cpp.o.requires

.PHONY : CMakeFiles/myslam.dir/requires

CMakeFiles/myslam.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/myslam.dir/cmake_clean.cmake
.PHONY : CMakeFiles/myslam.dir/clean

CMakeFiles/myslam.dir/depend:
	cd /home/giri/myslam/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/giri/myslam /home/giri/myslam /home/giri/myslam/build /home/giri/myslam/build /home/giri/myslam/build/CMakeFiles/myslam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/myslam.dir/depend

