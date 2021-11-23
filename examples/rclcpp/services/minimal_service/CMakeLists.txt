cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_service)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)

add_executable(service_main main.cpp)
ament_target_dependencies(service_main rclcpp example_interfaces)

add_executable(service_main_custom_srv main_custom_srv.cpp)
ament_target_dependencies(service_main_custom_srv rclcpp tutorial_interfaces)

install(TARGETS service_main service_main_custom_srv
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
