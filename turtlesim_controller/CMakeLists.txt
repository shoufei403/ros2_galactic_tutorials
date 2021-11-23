cmake_minimum_required(VERSION 3.8)
project(turtlesim_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(tutorial_interfaces REQUIRED)

add_executable(turtlesim_controller_node src/turtlesim_controller_node.cpp)
target_include_directories(turtlesim_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(turtlesim_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  turtlesim_controller_node
  "rclcpp"
  "std_msgs"
  "tutorial_interfaces"
  "turtlesim"
  "geometry_msgs"
)

add_executable(turtlesim_controller_client src/turtlesim_controller_client.cpp)
ament_target_dependencies(turtlesim_controller_client rclcpp std_msgs tutorial_interfaces)


install(TARGETS 
  turtlesim_controller_node
  turtlesim_controller_client
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
