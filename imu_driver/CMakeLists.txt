cmake_minimum_required(VERSION 3.8)
project(imu_driver)


add_executable(adis16470_node src/adis16470_node_ros2.cpp src/adis16470.cpp)
include_directories(include)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
ament_target_dependencies(adis16470_node rclcpp)

find_package(sensor_msgs REQUIRED)
ament_target_dependencies(adis16470_node sensor_msgs)

find_package(std_srvs REQUIRED)
ament_target_dependencies(adis16470_node std_srvs)


install(TARGETS adis16470_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  )

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
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
