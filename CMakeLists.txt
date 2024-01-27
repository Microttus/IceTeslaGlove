cmake_minimum_required(VERSION 3.5)
project(ice_tesla_glove)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

##Library
add_library(${CMAKE_PROJECT_NAME} STATIC
        src/ice_tesla_glove.cpp
        include/ice_tesla_glove/ice_tesla_glove.hh
)

ament_export_targets(${CMAKE_PROJECT_NAME} HAS_LIBRARY_TARGET)

target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        "$<BUILD_INTERFACE:${Serial_SOURCE_DIR}/include>"
        PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
)

install(DIRECTORY include/${CMAKE_PROJECT_NAME}
        DESTINATION include
        FILES_MATCHING PATTERN "*.hh"
)

# Export package in order to make an indeclinable library
install(TARGETS ${CMAKE_PROJECT_NAME}
        EXPORT ${CMAKE_PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)


ament_export_include_directories(include/${CMAKE_PROJECT_NAME})
ament_export_libraries(${CMAKE_PROJECT_NAME})
#ament_export_targets(${CMAKE_PROJECT_NAME})

ament_export_dependencies(ament_cmake
        rclcpp)

#add_executable(ice_glove src/ice_tesla_glove.cpp)
#ament_target_dependencies(ice_glove rclcpp)

#install(TARGETS
#ice_glove
#DESTINATION lib/${PROJECT_NAME})

ament_package()