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
CMAKE_SOURCE_DIR = /home/bruce/work/RAPTor/terrain_generator/hacd/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bruce/work/RAPTor/terrain_generator/hacd/build/linux2

# Include any dependencies generated for this target.
include test/CMakeFiles/testHACD.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/testHACD.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/testHACD.dir/flags.make

test/CMakeFiles/testHACD.dir/src/main.cpp.o: test/CMakeFiles/testHACD.dir/flags.make
test/CMakeFiles/testHACD.dir/src/main.cpp.o: /home/bruce/work/RAPTor/terrain_generator/hacd/src/test/src/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruce/work/RAPTor/terrain_generator/hacd/build/linux2/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object test/CMakeFiles/testHACD.dir/src/main.cpp.o"
	cd /home/bruce/work/RAPTor/terrain_generator/hacd/build/linux2/test && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/testHACD.dir/src/main.cpp.o -c /home/bruce/work/RAPTor/terrain_generator/hacd/src/test/src/main.cpp

test/CMakeFiles/testHACD.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testHACD.dir/src/main.cpp.i"
	cd /home/bruce/work/RAPTor/terrain_generator/hacd/build/linux2/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bruce/work/RAPTor/terrain_generator/hacd/src/test/src/main.cpp > CMakeFiles/testHACD.dir/src/main.cpp.i

test/CMakeFiles/testHACD.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testHACD.dir/src/main.cpp.s"
	cd /home/bruce/work/RAPTor/terrain_generator/hacd/build/linux2/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bruce/work/RAPTor/terrain_generator/hacd/src/test/src/main.cpp -o CMakeFiles/testHACD.dir/src/main.cpp.s

test/CMakeFiles/testHACD.dir/src/main.cpp.o.requires:
.PHONY : test/CMakeFiles/testHACD.dir/src/main.cpp.o.requires

test/CMakeFiles/testHACD.dir/src/main.cpp.o.provides: test/CMakeFiles/testHACD.dir/src/main.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/testHACD.dir/build.make test/CMakeFiles/testHACD.dir/src/main.cpp.o.provides.build
.PHONY : test/CMakeFiles/testHACD.dir/src/main.cpp.o.provides

test/CMakeFiles/testHACD.dir/src/main.cpp.o.provides.build: test/CMakeFiles/testHACD.dir/src/main.cpp.o

# Object files for target testHACD
testHACD_OBJECTS = \
"CMakeFiles/testHACD.dir/src/main.cpp.o"

# External object files for target testHACD
testHACD_EXTERNAL_OBJECTS =

test/testHACD: test/CMakeFiles/testHACD.dir/src/main.cpp.o
test/testHACD: test/CMakeFiles/testHACD.dir/build.make
test/testHACD: HACD_Lib/libHACD_LIB.a
test/testHACD: test/CMakeFiles/testHACD.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable testHACD"
	cd /home/bruce/work/RAPTor/terrain_generator/hacd/build/linux2/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testHACD.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/testHACD.dir/build: test/testHACD
.PHONY : test/CMakeFiles/testHACD.dir/build

test/CMakeFiles/testHACD.dir/requires: test/CMakeFiles/testHACD.dir/src/main.cpp.o.requires
.PHONY : test/CMakeFiles/testHACD.dir/requires

test/CMakeFiles/testHACD.dir/clean:
	cd /home/bruce/work/RAPTor/terrain_generator/hacd/build/linux2/test && $(CMAKE_COMMAND) -P CMakeFiles/testHACD.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/testHACD.dir/clean

test/CMakeFiles/testHACD.dir/depend:
	cd /home/bruce/work/RAPTor/terrain_generator/hacd/build/linux2 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bruce/work/RAPTor/terrain_generator/hacd/src /home/bruce/work/RAPTor/terrain_generator/hacd/src/test /home/bruce/work/RAPTor/terrain_generator/hacd/build/linux2 /home/bruce/work/RAPTor/terrain_generator/hacd/build/linux2/test /home/bruce/work/RAPTor/terrain_generator/hacd/build/linux2/test/CMakeFiles/testHACD.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/testHACD.dir/depend

