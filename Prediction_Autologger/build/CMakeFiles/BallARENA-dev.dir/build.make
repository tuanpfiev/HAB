# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/tony/HAB/Prediction_Autologger

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tony/HAB/Prediction_Autologger/build

# Include any dependencies generated for this target.
include CMakeFiles/BallARENA-dev.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/BallARENA-dev.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/BallARENA-dev.dir/flags.make

CMakeFiles/BallARENA-dev.dir/ballet.cpp.o: CMakeFiles/BallARENA-dev.dir/flags.make
CMakeFiles/BallARENA-dev.dir/ballet.cpp.o: ../ballet.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tony/HAB/Prediction_Autologger/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/BallARENA-dev.dir/ballet.cpp.o"
	/bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/BallARENA-dev.dir/ballet.cpp.o -c /home/tony/HAB/Prediction_Autologger/ballet.cpp

CMakeFiles/BallARENA-dev.dir/ballet.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BallARENA-dev.dir/ballet.cpp.i"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tony/HAB/Prediction_Autologger/ballet.cpp > CMakeFiles/BallARENA-dev.dir/ballet.cpp.i

CMakeFiles/BallARENA-dev.dir/ballet.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BallARENA-dev.dir/ballet.cpp.s"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tony/HAB/Prediction_Autologger/ballet.cpp -o CMakeFiles/BallARENA-dev.dir/ballet.cpp.s

# Object files for target BallARENA-dev
BallARENA__dev_OBJECTS = \
"CMakeFiles/BallARENA-dev.dir/ballet.cpp.o"

# External object files for target BallARENA-dev
BallARENA__dev_EXTERNAL_OBJECTS =

BallARENA-dev: CMakeFiles/BallARENA-dev.dir/ballet.cpp.o
BallARENA-dev: CMakeFiles/BallARENA-dev.dir/build.make
BallARENA-dev: CMakeFiles/BallARENA-dev.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tony/HAB/Prediction_Autologger/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable BallARENA-dev"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/BallARENA-dev.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/BallARENA-dev.dir/build: BallARENA-dev

.PHONY : CMakeFiles/BallARENA-dev.dir/build

CMakeFiles/BallARENA-dev.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/BallARENA-dev.dir/cmake_clean.cmake
.PHONY : CMakeFiles/BallARENA-dev.dir/clean

CMakeFiles/BallARENA-dev.dir/depend:
	cd /home/tony/HAB/Prediction_Autologger/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tony/HAB/Prediction_Autologger /home/tony/HAB/Prediction_Autologger /home/tony/HAB/Prediction_Autologger/build /home/tony/HAB/Prediction_Autologger/build /home/tony/HAB/Prediction_Autologger/build/CMakeFiles/BallARENA-dev.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/BallARENA-dev.dir/depend

