cmake_minimum_required(VERSION 3.0.2)
project(hero_chassis_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  controller_interface
  hardware_interface
  control_toolbox
  pluginlib
  geometry_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 关键：这里必须是 add_library，生成 .so 动态链接库
add_library(${PROJECT_NAME}
  src/hero_chassis_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
