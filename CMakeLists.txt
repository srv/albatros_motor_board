cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

# Default compile flags are "-W -Wall -Wno-unused-parameter -fno-strict-aliasing" 
# Uncomment the following line to turn on higher warnings
# set(ROS_COMPILE_FLAGS "-W -Wall -Wextra -pedantic")
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

include_directories(${PROJECT_SOURCE_DIR}/include/albatros_motor_board)

rosbuild_add_library(motor_board_ctrl
                       src/motor_board_ctrl/motor_board_ctrl.cpp
                       src/motor_board_ctrl/serial_comm.cpp
                       src/motor_board_ctrl/command_msg.cpp)

rosbuild_add_executable(motor_board_console
                          src/motor_board_console/motor_board_console.cpp
                          src/motor_board_console/simple_shell.cpp)
target_link_libraries(motor_board_console motor_board_ctrl)

rosbuild_add_executable(motor_board_node
                          src/motor_board_node.cpp
                          src/motor_board_node_base.cpp)
target_link_libraries(motor_board_node  motor_board_ctrl)

rosbuild_add_library(motor_board_nodelet
                       src/motor_board_nodelet.cpp
                       src/motor_board_node_base.cpp)
target_link_libraries(motor_board_nodelet motor_board_ctrl)
