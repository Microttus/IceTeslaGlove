cmake_minimum_required(VERSION 3.5)
project(ice_tesla_glove)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(${CMAKE_PROJECT_NAME} src/ice_tesla_glove.cpp)
ament_target_dependencies(${CMAKE_PROJECT_NAME} rclcpp std_msgs geometry_msgs)

install(TARGETS
        ${CMAKE_PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

ament_package()