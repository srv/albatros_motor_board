include_directories(${PROJECT_SOURCE_DIR}/include/albatros_motor_board)
rosbuild_add_executable(motor_board_console
                         motor_board_console.cpp
                         simple_shell.cpp
)
target_link_libraries(motor_board_console motor_board_ctrl)