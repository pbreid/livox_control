cmake_minimum_required(VERSION 3.0.2)
project(livox_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

# Specify the path to Livox SDK
set(LIVOX_SDK_PATH "/opt/ros/noetic")

# Find Livox SDK library
find_library(LIVOX_SDK_LIBRARY liblivox_sdk_static.a ${LIVOX_SDK_PATH}/lib)

if(NOT LIVOX_SDK_LIBRARY)
  message(FATAL_ERROR "Couldn't find Livox SDK library! Please install it.")
else()
  message(STATUS "Found Livox SDK library: ${LIVOX_SDK_LIBRARY}")
endif()

catkin_package(
  CATKIN_DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${LIVOX_SDK_PATH}/include
)

add_executable(${PROJECT_NAME}_node src/livox_control_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${LIVOX_SDK_LIBRARY}
  ${catkin_LIBRARIES}
)

# Ensure all Livox SDK symbols are available
target_compile_definitions(${PROJECT_NAME}_node PRIVATE LIVOX_SDK_STATIC)
