cmake_minimum_required(VERSION 3.1.3)
project(benchbot_multi_eef_moveit_configs)

find_package(catkin REQUIRED COMPONENTS
moveit_ros_planning_interface 
moveit_ros_move_group
rviz_visual_tools
moveit_visual_tools
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(moveit_cpp_interface src/moveit_cpp_interface.cpp)
target_link_libraries(moveit_cpp_interface ${catkin_LIBRARIES})
