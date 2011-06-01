include_directories(${PROJECT_SOURCE_DIR}/include/albatros_motorboard)
rosbuild_add_library(motorboardctrl
              motorboardctrl.cpp
              serialcomm.cpp
              commandmsg.cpp
)
