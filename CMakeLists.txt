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

# Specify the installation path for the 'tools' directory
set(tools_directory ${CMAKE_CURRENT_SOURCE_DIR}/tools)
set(install_tools_directory ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})

# Install the 'tools' directory
install(DIRECTORY ${tools_directory}
        DESTINATION ${install_tools_directory}
        USE_SOURCE_PERMISSIONS
        PATTERN "CMakeLists.txt" EXCLUDE  # Exclude CMakeLists.txt if it's in the 'tools' directory
)

add_executable(${CMAKE_PROJECT_NAME} src/ice_tesla_glove.cpp)
ament_target_dependencies(${CMAKE_PROJECT_NAME} rclcpp std_msgs geometry_msgs)

install(TARGETS
        ${CMAKE_PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
)



ament_package()