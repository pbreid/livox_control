cmake_minimum_required(VERSION 3.0.2)
project(livox_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

# Assuming Livox SDK is installed in a standard location
find_package(LivoxSDK REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/livox_control_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  LivoxSDK
)
