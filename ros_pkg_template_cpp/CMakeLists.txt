cmake_minimum_required(VERSION 3.5)
project(ros_pkg_template_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

add_library(ros_pkg_template_composition SHARED
        src/ros_pkg_template.cpp
        src/algorithm.cpp
        )

target_compile_definitions(ros_pkg_template_composition
  PRIVATE "ROS_PKG_TEMPLATE_DLL")
ament_target_dependencies(ros_pkg_template_composition rclcpp rclcpp_components std_msgs std_srvs rclcpp_lifecycle)

# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(ros_pkg_template_node src/ros_pkg_template_node.cpp)
target_link_libraries(ros_pkg_template_node ros_pkg_template_composition)
ament_target_dependencies(ros_pkg_template_node rclcpp class_loader)

install(TARGETS
  ros_pkg_template_composition
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  ros_pkg_template_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
        )

rclcpp_components_register_nodes(ros_pkg_template_composition "RosPkgTemplate")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
