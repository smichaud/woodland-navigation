# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o-build

# Include any dependencies generated for this target.
include g2o/stuff/CMakeFiles/opengl_helper.dir/depend.make

# Include the progress variables for this target.
include g2o/stuff/CMakeFiles/opengl_helper.dir/progress.make

# Include the compile flags for this target's objects.
include g2o/stuff/CMakeFiles/opengl_helper.dir/flags.make

g2o/stuff/CMakeFiles/opengl_helper.dir/opengl_primitives.cpp.o: g2o/stuff/CMakeFiles/opengl_helper.dir/flags.make
g2o/stuff/CMakeFiles/opengl_helper.dir/opengl_primitives.cpp.o: /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff/opengl_primitives.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o-build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object g2o/stuff/CMakeFiles/opengl_helper.dir/opengl_primitives.cpp.o"
	cd /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o-build/g2o/stuff && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/opengl_helper.dir/opengl_primitives.cpp.o -c /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff/opengl_primitives.cpp

g2o/stuff/CMakeFiles/opengl_helper.dir/opengl_primitives.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opengl_helper.dir/opengl_primitives.cpp.i"
	cd /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o-build/g2o/stuff && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff/opengl_primitives.cpp > CMakeFiles/opengl_helper.dir/opengl_primitives.cpp.i

g2o/stuff/CMakeFiles/opengl_helper.dir/opengl_primitives.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opengl_helper.dir/opengl_primitives.cpp.s"
	cd /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o-build/g2o/stuff && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff/opengl_primitives.cpp -o CMakeFiles/opengl_helper.dir/opengl_primitives.cpp.s

g2o/stuff/CMakeFiles/opengl_helper.dir/opengl_primitives.cpp.o.requires:
.PHONY : g2o/stuff/CMakeFiles/opengl_helper.dir/opengl_primitives.cpp.o.requires

g2o/stuff/CMakeFiles/opengl_helper.dir/opengl_primitives.cpp.o.provides: g2o/stuff/CMakeFiles/opengl_helper.dir/opengl_primitives.cpp.o.requires
	$(MAKE) -f g2o/stuff/CMakeFiles/opengl_helper.dir/build.make g2o/stuff/CMakeFiles/opengl_helper.dir/opengl_primitives.cpp.o.provides.build
.PHONY : g2o/stuff/CMakeFiles/opengl_helper.dir/opengl_primitives.cpp.o.provides

g2o/stuff/CMakeFiles/opengl_helper.dir/opengl_primitives.cpp.o.provides.build: g2o/stuff/CMakeFiles/opengl_helper.dir/opengl_primitives.cpp.o

# Object files for target opengl_helper
opengl_helper_OBJECTS = \
"CMakeFiles/opengl_helper.dir/opengl_primitives.cpp.o"

# External object files for target opengl_helper
opengl_helper_EXTERNAL_OBJECTS =

/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/lib/libg2o_opengl_helper.so: g2o/stuff/CMakeFiles/opengl_helper.dir/opengl_primitives.cpp.o
/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/lib/libg2o_opengl_helper.so: g2o/stuff/CMakeFiles/opengl_helper.dir/build.make
/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/lib/libg2o_opengl_helper.so: /usr/lib/x86_64-linux-gnu/libGLU.so
/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/lib/libg2o_opengl_helper.so: /usr/lib/x86_64-linux-gnu/libGL.so
/home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/lib/libg2o_opengl_helper.so: g2o/stuff/CMakeFiles/opengl_helper.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/lib/libg2o_opengl_helper.so"
	cd /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o-build/g2o/stuff && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/opengl_helper.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
g2o/stuff/CMakeFiles/opengl_helper.dir/build: /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/lib/libg2o_opengl_helper.so
.PHONY : g2o/stuff/CMakeFiles/opengl_helper.dir/build

g2o/stuff/CMakeFiles/opengl_helper.dir/requires: g2o/stuff/CMakeFiles/opengl_helper.dir/opengl_primitives.cpp.o.requires
.PHONY : g2o/stuff/CMakeFiles/opengl_helper.dir/requires

g2o/stuff/CMakeFiles/opengl_helper.dir/clean:
	cd /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o-build/g2o/stuff && $(CMAKE_COMMAND) -P CMakeFiles/opengl_helper.dir/cmake_clean.cmake
.PHONY : g2o/stuff/CMakeFiles/opengl_helper.dir/clean

g2o/stuff/CMakeFiles/opengl_helper.dir/depend:
	cd /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o/g2o/stuff /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o-build /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o-build/g2o/stuff /home/smichaud/Workspace/WoodlandNavigation/narfPlaceRecognition/EXTERNALS/ais3dTools/EXTERNALS/g2o/src/g2o-build/g2o/stuff/CMakeFiles/opengl_helper.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : g2o/stuff/CMakeFiles/opengl_helper.dir/depend

