include_directories(${PROJECT_SOURCE_DIR}/include/albatros_motor_board)
rosbuild_add_library(motor_board_ctrl
                      motor_board_ctrl.cpp
                      serial_comm.cpp
                      command_msg.cpp
)
