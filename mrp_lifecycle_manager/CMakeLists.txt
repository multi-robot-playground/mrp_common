cmake_minimum_required(VERSION 3.5)
project(mrp_lifecycle_manager)

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
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(mrp_common REQUIRED)
find_package(mrp_common_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(dependencies
  rclcpp
  rclcpp_lifecycle
  mrp_common
  mrp_common_msgs
  lifecycle_msgs
)

set(library_name mrp_lifecyle_manager)

add_library(${library_name} SHARED 
  src/lifecycle_manager.cpp
  src/lifecycle_manager_client.cpp
)

target_include_directories(mrp_lifecyle_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

##########################
# LIFECYCLE MANAGER EXEC #
##########################

set(lifecycle_manager lifecycle_manager)
add_executable(${lifecycle_manager}
  src/main.cpp
)

target_link_libraries(${lifecycle_manager} 
  ${library_name}
)

target_include_directories(${lifecycle_manager} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(${lifecycle_manager} ${dependencies})

install(TARGETS ${lifecycle_manager}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${library_name}
)

ament_package()
