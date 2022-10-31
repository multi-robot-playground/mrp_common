cmake_minimum_required(VERSION 3.5)
project(mrp_common)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mrp_common_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
# include_directories(
#   include
# )

set(dependencies
  rclcpp
  rclcpp_components
  mrp_common_msgs
  rclcpp_lifecycle
  lifecycle_msgs
)

set(HEADER_FILES 
  include/mrp_common/action_client.hpp
  include/mrp_common/action_server.hpp
  include/mrp_common/lifecycle_node.hpp
  include/mrp_common/service_client.hpp
  include/mrp_common/service_server.hpp
)

set(library_name mrp_common)
add_library(${library_name} SHARED
  src/lifecycle_node.cpp
  src/utils.cpp
  ${HEADER_FILES}
)

target_include_directories(${library_name} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)


ament_target_dependencies(${library_name} ${dependencies})

install(
  TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# ament_export_targets(${library_name} HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/${library_name}
  DESTINATION include
)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Heartbeat.msg"
#   DEPENDENCIES std_msgs
# )

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

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_package()
