cmake_minimum_required(VERSION 3.8)
project(laser_trajetory_points)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake        REQUIRED)
find_package(geometry_msgs      REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(std_srvs           REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(rclcpp_lifecycle   REQUIRED)
find_package(rclcpp_components  REQUIRED)


set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  std_srvs
  geometry_msgs
)

set(executable_name laser_trajectory_points)
set(library_name ${executable_name}_core)

add_executable(${executable_name}
  src/laser_trajectory_points_main.cpp
)

add_library(${library_name} SHARED
  src/laser_trajectory_points_node.cpp
)

target_include_directories(${library_name} PRIVATE src/include)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name}
  ${library_name}
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "laser_trajectory_points::laser_trajectory_points_node")

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME} 
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_package()
