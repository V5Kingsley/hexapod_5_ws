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
CMAKE_SOURCE_DIR = /home/sun/hexapod_5_ws/src/my_hexapod_control

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sun/hexapod_5_ws/src/my_hexapod_control/build

# Include any dependencies generated for this target.
include CMakeFiles/my_control.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/my_control.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/my_control.dir/flags.make

CMakeFiles/my_control.dir/src/control.cpp.o: CMakeFiles/my_control.dir/flags.make
CMakeFiles/my_control.dir/src/control.cpp.o: ../src/control.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sun/hexapod_5_ws/src/my_hexapod_control/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/my_control.dir/src/control.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/my_control.dir/src/control.cpp.o -c /home/sun/hexapod_5_ws/src/my_hexapod_control/src/control.cpp

CMakeFiles/my_control.dir/src/control.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my_control.dir/src/control.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sun/hexapod_5_ws/src/my_hexapod_control/src/control.cpp > CMakeFiles/my_control.dir/src/control.cpp.i

CMakeFiles/my_control.dir/src/control.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my_control.dir/src/control.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sun/hexapod_5_ws/src/my_hexapod_control/src/control.cpp -o CMakeFiles/my_control.dir/src/control.cpp.s

CMakeFiles/my_control.dir/src/control.cpp.o.requires:
.PHONY : CMakeFiles/my_control.dir/src/control.cpp.o.requires

CMakeFiles/my_control.dir/src/control.cpp.o.provides: CMakeFiles/my_control.dir/src/control.cpp.o.requires
	$(MAKE) -f CMakeFiles/my_control.dir/build.make CMakeFiles/my_control.dir/src/control.cpp.o.provides.build
.PHONY : CMakeFiles/my_control.dir/src/control.cpp.o.provides

CMakeFiles/my_control.dir/src/control.cpp.o.provides.build: CMakeFiles/my_control.dir/src/control.cpp.o

# Object files for target my_control
my_control_OBJECTS = \
"CMakeFiles/my_control.dir/src/control.cpp.o"

# External object files for target my_control
my_control_EXTERNAL_OBJECTS =

../devel/lib/libmy_control.so: CMakeFiles/my_control.dir/src/control.cpp.o
../devel/lib/libmy_control.so: CMakeFiles/my_control.dir/build.make
../devel/lib/libmy_control.so: CMakeFiles/my_control.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library ../devel/lib/libmy_control.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/my_control.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/my_control.dir/build: ../devel/lib/libmy_control.so
.PHONY : CMakeFiles/my_control.dir/build

CMakeFiles/my_control.dir/requires: CMakeFiles/my_control.dir/src/control.cpp.o.requires
.PHONY : CMakeFiles/my_control.dir/requires

CMakeFiles/my_control.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/my_control.dir/cmake_clean.cmake
.PHONY : CMakeFiles/my_control.dir/clean

CMakeFiles/my_control.dir/depend:
	cd /home/sun/hexapod_5_ws/src/my_hexapod_control/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sun/hexapod_5_ws/src/my_hexapod_control /home/sun/hexapod_5_ws/src/my_hexapod_control /home/sun/hexapod_5_ws/src/my_hexapod_control/build /home/sun/hexapod_5_ws/src/my_hexapod_control/build /home/sun/hexapod_5_ws/src/my_hexapod_control/build/CMakeFiles/my_control.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/my_control.dir/depend
