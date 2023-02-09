cmake_minimum_required(VERSION 3.8)
project(ati_sensor_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# find soem
find_package(soem REQUIRED)
find_library(soem_lib REQUIRED
  NAMES soem
)
find_path(soem_include_dir REQUIRED
  NAMES soem/ethercat.h
)

add_executable(${PROJECT_NAME}
  src/ati_sensor_ros.cpp
  src/ati_sensor_lib.cpp
  # src/test_sensor.cpp
)
target_include_directories(${PROJECT_NAME} 
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${soem_include_dir}
)
target_link_libraries(${PROJECT_NAME}
  ${soem_lib}
)
ament_target_dependencies(${PROJECT_NAME} 
  rclcpp
  geometry_msgs
  std_srvs
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
