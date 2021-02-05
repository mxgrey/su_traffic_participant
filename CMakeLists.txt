cmake_minimum_required(VERSION 3.5)
project(su_traffic_participant)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
include(GNUInstallDirs)

set(dep_pkgs
  rclcpp
  rmf_utils
  rmf_traffic
  rmf_traffic_ros2
  su_msgs
  std_msgs
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()


file(GLOB_RECURSE traffic_writer_srcs 
  src/WriterNode.cpp)

add_library(traffic_writer SHARED
  ${traffic_writer_srcs}
)

target_link_libraries(traffic_writer
  PUBLIC
    rmf_traffic_ros2::rmf_traffic_ros2
    rmf_traffic::rmf_traffic
    ${su_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
)

target_include_directories(traffic_writer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${su_msgs_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
)

#===============================================================================

add_executable(writer_node src/WriterNode.cpp)

target_link_libraries(writer_node
  PUBLIC
    rmf_traffic_ros2::rmf_traffic_ros2
    rmf_traffic::rmf_traffic
    ${su_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
)

target_include_directories(writer_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${su_msgs_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
)

#===============================================================================

add_executable(listener src/sum_subscriber.cpp)

target_link_libraries(listener
  PUBLIC
    rmf_traffic_ros2::rmf_traffic_ros2
    rmf_traffic::rmf_traffic
    ${su_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
  PRIVATE
    traffic_writer
)

target_include_directories(listener
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${su_msgs_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
)

#===============================================================================

install(
  TARGETS
    traffic_writer
    writer_node
    listener
  EXPORT su_traffic_participant
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)


#===============================================================================

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
