cmake_minimum_required(VERSION 3.5)
project(je_arm_pcb_inspection_sm)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(common REQUIRED)
find_package(cl_ros2_timer REQUIRED)
find_package(smacc2 REQUIRED)
find_package(smacc2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(cl_moveit2z REQUIRED)
find_package(cl_keyboard REQUIRED)
find_package(my_robot_arm_cb_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  cl_ros2_timer
  smacc2
  smacc2_msgs
  std_msgs
  cl_moveit2z
  cl_keyboard
  common
  my_robot_arm_cb_library
  geometry_msgs
  moveit_msgs
  rclcpp
  shape_msgs
  ament_index_cpp
)

include_directories(
  include
  ${smacc2_INCLUDE_DIRS}
  ${moveit2z_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/je_arm_pcb_inspection_sm/je_arm_pcb_inspection_sm_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${smacc2_LIBRARIES}
  ${cl_ros2_timer_LIBRARIES}
  ${moveit2z_LIBRARIES}
  ${Boost_LIBRARIES}
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  rviz
  scripts
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/ethercat_registers
  scripts/ethercat_clear_registers
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
