cmake_minimum_required(VERSION 3.8)
project(som_cws_ajrp)

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

set(ament_dependencies
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(coordinate_publisher src/coordinate_publisher.cpp)
add_executable(distance_calculator src/distance_calculator.cpp)
add_executable(tsp_solver src/tsp_solver.cpp)


ament_target_dependencies(coordinate_publisher ${ament_dependencies} )
ament_target_dependencies(distance_calculator ${ament_dependencies} )
ament_target_dependencies(tsp_solver ${ament_dependencies} )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
coordinate_publisher
distance_calculator
tsp_solver
DESTINATION lib/${PROJECT_NAME})

ament_package()