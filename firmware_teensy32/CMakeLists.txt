cmake_minimum_required(VERSION 3.1)

# Compile as C++14.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)  # GNU extensions required to compile TeensyCore library.

include_directories(${ROS_LIB_DIR})

FILE(GLOB_RECURSE ros_src
  "${ROS_LIB_DIR}/*.cpp"
  "${ROS_LIB_DIR}/*.h")
add_library(ros_lib ${ros_src})

add_teensy_executable(earth_rover_firmware_teensy32 main.cpp)
target_link_libraries(earth_rover_firmware_teensy32 ros_lib)

