cmake_minimum_required(VERSION 2.8.3)
project(albatros_motor_board)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs driver_base dynamic_reconfigure auv_sensor_msgs auv_control_msgs nodelet message_generation)

add_message_files(
  DIRECTORY
  msg
  FILES
  MotorStatus.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

generate_dynamic_reconfigure_options(cfg/MotorBoardDynParams.cfg)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
  LIBRARIES motor_board_ctrl)

# <cpp lflags="-Wl,-rpath,${prefix}/lib -L${prefix}/lib -lmotor_board_ctrl" cflags="-I${prefix}/include"/>

include_directories(include cfg/cpp ${catkin_INCLUDE_DIRECTORIES})

add_library(motor_board_ctrl
            src/motor_board_ctrl/motor_board_ctrl.cpp
            src/motor_board_ctrl/serial_comm.cpp
            src/motor_board_ctrl/command_msg.cpp)

add_executable(motor_board_console
               src/motor_board_console/motor_board_console.cpp
               src/motor_board_console/simple_shell.cpp)
target_link_libraries(motor_board_console motor_board_ctrl ${catkin_LIBRARIES})

add_executable(motor_board_node
               src/motor_board_node.cpp
               src/motor_board_node_base.cpp)
target_link_libraries(motor_board_node  motor_board_ctrl ${catkin_LIBRARIES})

add_library(motor_board_nodelet
            src/motor_board_nodelet.cpp
            src/motor_board_node_base.cpp)
target_link_libraries(motor_board_nodelet motor_board_ctrl ${catkin_LIBRARIES})
