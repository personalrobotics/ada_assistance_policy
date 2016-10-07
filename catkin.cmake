# First, set up the ROS Catkin package settings.  
# This will find a virtual package called 'catkin' that contains includes 
# and libraries aggregated from all the ROS catkin packages you depend on.
# 
# This macro creates:
# catkin_INCLUDE_DIRS, catkin_LIBRARIES, catkin_LIBRARY_DIRS

find_package(catkin REQUIRED COMPONENTS
                    rospy
            )
#                    prpy
#                   or_cdchomp
#                    cbirrt2
#            )
 
# For system dependencies, use CMake's 'find_package' macros.
# These macros typically create (for a package named 'foo'):
# foo_INCLUDE_DIRS, foo_LIBRARIES, foo_LIBRARY_DIRS
find_package(openrave REQUIRED)
 
# Set up the ROS Catkin package settings.
catkin_package()
 
# Some system dependencies don't have 'find_package' macros.  For these
# packages, CMake includes a helper function which can resolve the libraries
# using pkg-config, which most libraries support, and catkin builds generate.
#include(FindPkgConfig)
#pkg_check_modules(Yaml REQUIRED yaml-cpp)
 
# Add whatever weird compilation options you need.
#add_definitions(-frounding-math)
 
# If you are packaging a third-party repository, you can use ExternalProject 
# to download and build it within your wrapper.
#include(ExternalProject)
#ExternalProject_Add(apriltags_swatbotics_EXTERNAL
#    GIT_REPOSITORY https://github.com/personalrobotics/apriltags-cpp
#    INSTALL_COMMAND ""
#    BUILD_COMMAND "make" # I don't know why this works...
#    CMAKE_ARGS -DCMAKE_CXX_FLAGS=-frounding-math -DBUILD_SHARED_LIBS:BOOL=ON
#)
 
# If you are using ExternalProject, you need some additional magic to use
# your imported source code as a build target.
#ExternalProject_Get_Property(apriltags_swatbotics_EXTERNAL
#  SOURCE_DIR BINARY_DIR INSTALL_DIR)
#set(apriltags_swatbotics_INCLUDE_DIRS
#  "${SOURCE_DIR}"
#)
#set(apriltags_swatbotics_LIBRARIES
#  "${BINARY_DIR}/libapriltags.so"
#)
# Tell CMake that the external project generated a library so we
# can add dependencies here instead of later.
#add_library(apriltags_swatbotics UNKNOWN IMPORTED)
#set_property(TARGET apriltags_swatbotics
#  PROPERTY IMPORTED_LOCATION
#  ${apriltags_swatbotics_LIBRARIES}
#)
#add_dependencies(apriltags_swatbotics apriltags_swatbotics_EXTERNAL)
 
# Add ALL the includes we need to build: stuff from catkin AND system dependencies.
include_directories(
    include/
    ${catkin_INCLUDE_DIRS}
    ${openrave_INCLUDE_DIRS}
)

catkin_python_setup()
 
# CMake has add_executable and add_library functions to define build 'targets'.
#add_executable(apriltags src/apriltags.cpp)
 
# For each 'target', specify all the libraries it will need to be linked against.
#target_link_libraries(apriltags ${catkin_LIBRARIES})
#target_link_libraries(apriltags ${catkin_LIBRARIES})
#target_link_libraries(apriltags ${Eigen_LIBRARIES})
#target_link_libraries(apriltags ${OpenCV_LIBRARIES})
#target_link_libraries(apriltags ${Yaml_LIBRARIES})
#target_link_libraries(apriltags apriltags_swatbotics)
 
# At the end of the build, tell catkin to INSTALL your 'target'.
#install(TARGETS adapy
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
#)
 
# Sometimes you create files that aren't specific to a target or just need
# to be copied.  You can use this version of the install command for that.
#install(FILES ${apriltags_swatbotics_LIBRARIES}
#  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)
